#include "SkyboundAircraft.h"

#include "Components/InputComponent.h"
#include "Components/StaticMeshComponent.h"

namespace SkyboundFlight
{
    // Unreal distance is centimetres; aerodynamic calculations use SI units.
    constexpr float CmPerMeter = 100.0f;
    constexpr float NewtonToUnrealForce = 100.0f;
    constexpr float NewtonMeterToUnrealTorque = 10000.0f;
    constexpr float DegToRad = PI / 180.0f;
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

    const FVector VelocityMS =
        AircraftMesh->GetPhysicsLinearVelocity() / SkyboundFlight::CmPerMeter;

    AirspeedMetersPerSecond = VelocityMS.Size();
    AirspeedKilometersPerHour = AirspeedMetersPerSecond * 3.6f;
    VerticalSpeedMetersPerSecond = VelocityMS.Z;

    // Unreal's rigid-body simulation already applies gravity.
    const FVector ThrustN =
        GetActorForwardVector() * (MaximumThrustNewtons * Throttle);
    AircraftMesh->AddForce(ThrustN * SkyboundFlight::NewtonToUnrealForce);

    UpdateAerodynamics(DeltaTime);
    UpdateFlightControls(DeltaTime);
}

void ASkyboundAircraft::UpdateAerodynamics(float DeltaTime)
{
    (void)DeltaTime;

    const FVector VelocityMS =
        AircraftMesh->GetPhysicsLinearVelocity() / SkyboundFlight::CmPerMeter;
    const float SpeedMS = VelocityMS.Size();

    bStalled = false;
    AngleOfAttackDegrees = 0.0f;

    if (SpeedMS < SkyboundFlight::MinimumAirspeed)
    {
        return;
    }

    const FVector LocalVelocity =
        AircraftMesh->GetComponentTransform().InverseTransformVectorNoScale(VelocityMS);

    // Aircraft local axes: +X forward, +Y right, +Z up.
    const float AlphaRad = FMath::Atan2(
        -LocalVelocity.Z,
        FMath::Max(0.1f, LocalVelocity.X)
    );
    AngleOfAttackDegrees = AlphaRad / SkyboundFlight::DegToRad;

    const float AltitudeM =
        FMath::Max(0.0f, GetActorLocation().Z / SkyboundFlight::CmPerMeter);
    const float Density =
        SeaLevelAirDensity * FMath::Exp(-AltitudeM / AtmosphericScaleHeightMeters);
    const float DynamicPressure = 0.5f * Density * SpeedMS * SpeedMS;

    // Elevator input changes effective wing incidence in this first-pass model.
    const float EffectiveAlpha = AlphaRad + PitchInput * ControlSurfaceLiftAuthority;
    const float CriticalAlpha =
        CriticalAngleOfAttackDegrees * SkyboundFlight::DegToRad;
    const float AbsAlpha = FMath::Abs(EffectiveAlpha);
    const float AlphaSign = FMath::Sign(EffectiveAlpha);

    float LiftCoefficient = LiftCurveSlopePerRadian * EffectiveAlpha;

    // Beyond critical AoA, lift decays and drag rises to produce a recoverable
    // game-model stall instead of allowing unlimited lift at high pitch.
    if (AbsAlpha > CriticalAlpha)
    {
        bStalled = true;
        const float PeakLift = LiftCurveSlopePerRadian * CriticalAlpha;
        LiftCoefficient = AlphaSign * PeakLift
            * FMath::Exp(-(AbsAlpha - CriticalAlpha) * 2.0f);
    }

    LiftCoefficient = FMath::Clamp(LiftCoefficient, -1.6f, 1.6f);

    const FVector AirflowDirection = VelocityMS.GetSafeNormal();
    FVector LiftDirection =
        FVector::VectorPlaneProject(GetActorUpVector(), AirflowDirection).GetSafeNormal();

    if (LiftDirection.IsNearlyZero())
    {
        LiftDirection =
            FVector::VectorPlaneProject(GetActorRightVector(), AirflowDirection).GetSafeNormal();
    }

    if (!LiftDirection.IsNearlyZero())
    {
        const float LiftN =
            DynamicPressure * WingAreaSquareMeters * LiftCoefficient;
        AircraftMesh->AddForce(
            LiftDirection * LiftN * SkyboundFlight::NewtonToUnrealForce
        );
    }

    const float DragCoefficient =
        ZeroLiftDragCoefficient
        + InducedDragFactor * FMath::Square(LiftCoefficient)
        + (bStalled ? 0.18f : 0.0f);

    const float DragN =
        DynamicPressure * WingAreaSquareMeters * DragCoefficient;
    AircraftMesh->AddForce(
        -AirflowDirection * DragN * SkyboundFlight::NewtonToUnrealForce
    );
}

void ASkyboundAircraft::UpdateFlightControls(float DeltaTime)
{
    (void)DeltaTime;

    const FVector AngularVelocityLocal =
        AircraftMesh->GetComponentTransform().InverseTransformVectorNoScale(
            AircraftMesh->GetPhysicsAngularVelocityInRadians()
        );

    const float SpeedMS =
        AircraftMesh->GetPhysicsLinearVelocity().Size() / SkyboundFlight::CmPerMeter;
    const float Authority = FMath::Clamp(SpeedMS / 35.0f, 0.0f, 1.0f);
    const float ControlScale = FMath::Max(0.12f, Authority * Authority);

    // Local torque axes: X roll, Y pitch, Z yaw.
    // With +X forward, negative Y raises the nose.
    const float RollNm =
        -RollInput * RollControlMoment * ControlScale
        - AngularVelocityLocal.X * RollRateDamping;

    const float PitchNm =
        -PitchInput * PitchControlMoment * ControlScale
        - AngularVelocityLocal.Y * PitchRateDamping;

    const float YawNm =
        YawInput * YawControlMoment * ControlScale
        - AngularVelocityLocal.Z * YawRateDamping;

    const FVector TorqueNm(RollNm, PitchNm, YawNm);
    AircraftMesh->AddTorqueInRadians(
        TorqueNm * SkyboundFlight::NewtonMeterToUnrealTorque
    );
}