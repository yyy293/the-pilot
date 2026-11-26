#include "SkyboundAircraft.h"

#include "Components/StaticMeshComponent.h"
#include "GameFramework/PlayerController.h"
#include "Components/InputComponent.h"

namespace SkyboundFlight
{
    // Unreal uses centimetres for distance. Aerodynamic calculations below
    // use SI units, then convert N -> kg*cm/s^2 and N*m -> kg*cm^2/s^2.
    constexpr float MetersToCentimeters = 100.0f;
    constexpr float NewtonsToUnrealForce = 100.0f;
    constexpr float NewtonMetersToUnrealTorque = 10000.0f;
    constexpr float DegreesToRadians = PI / 180.0f;
    constexpr float MinimumAirspeed = 0.5f;
}

ASkyboundAircraft::ASkyboundAircraft()
{
    PrimaryActorTick.bCanEverTick = true;

    AircraftMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("AircraftMesh"));
    SetRootComponent(AircraftMesh);

    AircraftMesh->SetSimulatePhysics(true);
    AircraftMesh->SetEnableGravity(true);
    AircraftMesh->SetLinearDamping(0.0f);
    AircraftMesh->SetAngularDamping(0.05f);
    AircraftMesh->SetCollisionProfileName(TEXT("PhysicsActor"));
    AircraftMesh->SetMassOverrideInKg(NAME_None, AircraftMassKg, true);

    AutoPossessPlayer = EAutoReceiveInput::Player0;
}

void ASkyboundAircraft::BeginPlay()
{
    Super::BeginPlay();

    if (AircraftMesh)
    {
        AircraftMesh->SetMassOverrideInKg(NAME_None, AircraftMassKg, true);
        AircraftMesh->WakeAllRigidBodies();
    }
}

void ASkyboundAircraft::SetThrottleAxis(float Value)
{
    ThrottleAxis = FMath::Clamp(Value, -1.0f, 1.0f);
}

void ASkyboundAircraft::SetPitch(float Value)
{
    PitchInput = FMath::Clamp(Value, -1.0f, 1.0f);
}

void ASkyboundAircraft::SetRoll(float Value)
{
    RollInput = FMath::Clamp(Value, -1.0f, 1.0f);
}

void ASkyboundAircraft::SetYaw(float Value)
{
    YawInput = FMath::Clamp(Value, -1.0f, 1.0f);
}

void ASkyboundAircraft::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    check(PlayerInputComponent);

    PlayerInputComponent->BindAxis(TEXT("Pitch"), this, &ASkyboundAircraft::SetPitch);
    PlayerInputComponent->BindAxis(TEXT("Roll"), this, &ASkyboundAircraft::SetRoll);
    PlayerInputComponent->BindAxis(TEXT("Yaw"), this, &ASkyboundAircraft::SetYaw);
    PlayerInputComponent->BindAxis(TEXT("Throttle"), this, &ASkyboundAircraft::SetThrottleAxis);
}

void ASkyboundAircraft::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (!AircraftMesh || !AircraftMesh->IsSimulatingPhysics() || DeltaTime <= 0.0f)
    {
        return;
    }

    Throttle = FMath::Clamp(
        Throttle + ThrottleAxis * ThrottleChangeRate * DeltaTime,
        0.0f,
        1.0f
    );

    UpdateAerodynamics(DeltaTime);
    UpdateFlightControls(DeltaTime);

    const FVector VelocityCmPerSecond = AircraftMesh->GetPhysicsLinearVelocity();
    const FVector VelocityMetersPerSecond = VelocityCmPerSecond / SkyboundFlight::MetersToCentimeters;

    AirspeedMetersPerSecond = VelocityMetersPerSecond.Size();
    AirspeedKilometersPerHour = AirspeedMetersPerSecond * 3.6f;
    VerticalSpeedMetersPerSecond = VelocityMetersPerSecond.Z;

    const FVector WorldUp = FVector::UpVector;
    const float AltitudeMeters = FMath::Max(0.0f, GetActorLocation().Z / SkyboundFlight::MetersToCentimeters);
    const float Density = SeaLevelAirDensity * FMath::Exp(-AltitudeMeters / AtmosphericScaleHeightMeters);

    // Thrust follows the aircraft's forward axis. This is a simplified
    // constant-thrust piston-engine model, not a propeller blade simulation.
    const FVector ThrustNewtons = GetActorForwardVector() * (MaximumThrustNewtons * Throttle);
    AircraftMesh->AddForce(ThrustNewtons * SkyboundFlight::NewtonsToUnrealForce);

    // Gravity is supplied by Unreal's rigid-body simulation; do not add it twice.
    (void)WorldUp;
    (void)Density;
}

void ASkyboundAircraft::UpdateAerodynamics(float DeltaTime)
{
    (void)DeltaTime;

    const FVector VelocityWorldCm = AircraftMesh->GetPhysicsLinearVelocity();
    const FVector VelocityWorld = VelocityWorldCm / SkyboundFlight::MetersToCentimeters;
    const float Speed = VelocityWorld.Size();

    bStalled = false;
    AngleOfAttackDegrees = 0.0f;

    if (Speed < SkyboundFlight::MinimumAirspeed)
    {
        return;
    }

    const FTransform AircraftTransform = AircraftMesh->GetComponentTransform();
    const FVector LocalVelocity = AircraftTransform.InverseTransformVectorNoScale(VelocityWorld);
    const float ForwardSpeed = LocalVelocity.X;

    // Angle of attack is measured from the local airflow direction.
    const float Alpha = FMath::Atan2(-LocalVelocity.Z, FMath::Max(0.1f, ForwardSpeed));
    AngleOfAttackDegrees = Alpha / SkyboundFlight::DegreesToRadians;

    const float AltitudeMeters = FMath::Max(0.0f, GetActorLocation().Z / SkyboundFlight::MetersToCentimeters);
    const float Density = SeaLevelAirDensity * FMath::Exp(-AltitudeMeters / AtmosphericScaleHeightMeters);
    const float DynamicPressure = 0.5f * Density * Speed * Speed;

    const float CriticalAlpha = CriticalAngleOfAttackDegrees * SkyboundFlight::DegreesToRadians;
    const float EffectiveAlpha = Alpha + PitchInput * ControlSurfaceLiftAuthority;

    // Linear lift before stall; after the critical angle, lift smoothly
    // decays instead of remaining unrealistically clamped at maximum.
    const float AbsoluteAlpha = FMath::Abs(EffectiveAlpha);
    const float Sign = FMath::Sign(EffectiveAlpha);
    float LiftCoefficient = LiftCurveSlopePerRadian * EffectiveAlpha;

    if (AbsoluteAlpha > CriticalAlpha)
    {
        bStalled = true;
        const float BeyondStall = AbsoluteAlpha - CriticalAlpha;
        const float PeakLift = LiftCurveSlopePerRadian * CriticalAlpha;
        const float PostStallLift = PeakLift * FMath::Exp(-BeyondStall * 2.0f);
        LiftCoefficient = Sign * PostStallLift;
    }

    LiftCoefficient = FMath::Clamp(LiftCoefficient, -1.6f, 1.6f);

    // Lift points perpendicular to the relative airflow, toward the aircraft's
    // upper side. This keeps lift direction correct during climbs and descents.
    const FVector AirflowDirection = VelocityWorld.GetSafeNormal();
    FVector LiftDirection = FVector::VectorPlaneProject(GetActorUpVector(), AirflowDirection).GetSafeNormal();

    if (LiftDirection.IsNearlyZero())
    {
        LiftDirection = FVector::VectorPlaneProject(GetActorRightVector(), AirflowDirection).GetSafeNormal();
    }

    const float LiftNewtons = DynamicPressure * WingAreaSquareMeters * LiftCoefficient;
    AircraftMesh->AddForce(LiftDirection * LiftNewtons * SkyboundFlight::NewtonsToUnrealForce);

    // Parasitic + induced drag. Add extra drag in the post-stall region.
    float DragCoefficient = ZeroLiftDragCoefficient
        + InducedDragFactor * FMath::Square(LiftCoefficient);

    if (bStalled)
    {
        DragCoefficient += 0.18f;
    }

    const float DragNewtons = DynamicPressure * WingAreaSquareMeters * DragCoefficient;
    AircraftMesh->AddForce(-AirflowDirection * DragNewtons * SkyboundFlight::NewtonsToUnrealForce);
}

void ASkyboundAircraft::UpdateFlightControls(float DeltaTime)
{
    (void)DeltaTime;

    const FVector AngularVelocityWorld = AircraftMesh->GetPhysicsAngularVelocityInRadians();
    const FVector AngularVelocityLocal =
        AircraftMesh->GetComponentTransform().InverseTransformVectorNoScale(AngularVelocityWorld);

    const float Speed = AircraftMesh->GetPhysicsLinearVelocity().Size() / SkyboundFlight::MetersToCentimeters;
    const float Authority = FMath::Clamp(Speed / 35.0f, 0.0f, 1.0f);

    // Control moments grow with airspeed, but remain controllable near the ground.
    const float DynamicControlScale = FMath::Max(0.12f, Authority * Authority);

    // Unreal local axes: X forward, Y right, Z up.
    // Positive pitch input means nose up; positive roll means right bank.
    const float PitchMomentNm =
        (PitchInput * PitchControlMoment * DynamicControlScale)
        - (AngularVelocityLocal.Y * PitchRateDamping);

    const float RollMomentNm =
        (-RollInput * RollControlMoment * DynamicControlScale)
        - (AngularVelocityLocal.X * RollRateDamping);

    const float YawMomentNm =
        (YawInput * YawControlMoment * DynamicControlScale)
        - (AngularVelocityLocal.Z * YawRateDamping);

    const FVector LocalTorqueNm(PitchMomentNm == 0.0f ? 0.0f : 0.0f, -PitchMomentNm, YawMomentNm);
    const FVector TorqueNm(
        RollMomentNm,
        -PitchMomentNm,
        YawMomentNm
    );

    AircraftMesh->AddTorqueInRadians(TorqueNm * SkyboundFlight::NewtonMetersToUnrealTorque);
}