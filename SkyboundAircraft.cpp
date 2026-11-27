#include "SkyboundAircraft.h"

#include "Components/InputComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"

namespace SkyboundFlight
{
    // Unreal uses centimetres and kilograms. Aerodynamic calculations below
    // use SI units, so convert N -> kg*cm/s^2 and N*m -> kg*cm^2/s^2.
    constexpr float CmPerMeter = 100.0f;
    constexpr float NewtonToUnrealForce = 100.0f;
    constexpr float NewtonMeterToUnrealTorque = 10000.0f;
    constexpr float DegToRad = PI / 180.0f;
    constexpr float MinimumAirspeedMS = 1.0f;
    constexpr float ReferenceControlSpeedMS = 45.0f;
}

ASkyboundAircraft::ASkyboundAircraft()
{
    PrimaryActorTick.bCanEverTick = true;

    AircraftMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("AircraftMesh"));
    SetRootComponent(AircraftMesh);

    // The GLB is assigned through AircraftModel in a Blueprint derived from
    // this pawn. Enable rigid-body simulation after the mesh is assigned.
    AircraftMesh->SetSimulatePhysics(false);
    AircraftMesh->SetEnableGravity(true);
    AircraftMesh->SetLinearDamping(0.0f);
    AircraftMesh->SetAngularDamping(0.0f);
    AircraftMesh->SetCollisionProfileName(TEXT("PhysicsActor"));

    AutoPossessPlayer = EAutoReceiveInput::Player0;
}

void ASkyboundAircraft::BeginPlay()
{
    Super::BeginPlay();

    if (!AircraftMesh)
    {
        return;
    }

    if (AircraftModel)
    {
        AircraftMesh->SetStaticMesh(AircraftModel);
    }

    if (AircraftMesh->GetStaticMesh())
    {
        AircraftMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        AircraftMesh->SetMassOverrideInKg(NAME_None, AircraftMassKg, true);
        AircraftMesh->SetSimulatePhysics(true);
        AircraftMesh->WakeAllRigidBodies();
    }
    else
    {
        UE_LOG(LogTemp, Warning,
            TEXT("SKYBOUND aircraft has no Static Mesh. Assign AircraftModel in the aircraft Blueprint."));
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

FVector ASkyboundAircraft::GetAirRelativeVelocityMS() const
{
    if (!AircraftMesh)
    {
        return FVector::ZeroVector;
    }

    const FVector GroundVelocityMS =
        AircraftMesh->GetPhysicsLinearVelocity() / SkyboundFlight::CmPerMeter;

    return GroundVelocityMS - WindVelocityMetersPerSecond;
}

float ASkyboundAircraft::GetAirDensityKgPerM3() const
{
    const float AltitudeMeters =
        FMath::Max(0.0f, GetActorLocation().Z / SkyboundFlight::CmPerMeter);

    return SeaLevelAirDensity *
        FMath::Exp(-AltitudeMeters / AtmosphericScaleHeightMeters);
}

float ASkyboundAircraft::GetDynamicPressurePa() const
{
    const float SpeedMS = GetAirRelativeVelocityMS().Size();
    return 0.5f * GetAirDensityKgPerM3() * SpeedMS * SpeedMS;
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

    const FVector AirVelocityMS = GetAirRelativeVelocityMS();
    AirspeedMetersPerSecond = AirVelocityMS.Size();
    AirspeedKilometersPerHour = AirspeedMetersPerSecond * 3.6f;
    VerticalSpeedMetersPerSecond =
        AircraftMesh->GetPhysicsLinearVelocity().Z / SkyboundFlight::CmPerMeter;
    DynamicPressurePascals = GetDynamicPressurePa();

    // A simple propeller model: thrust falls with forward speed rather than
    // remaining a constant force at every airspeed.
    const float SpeedRatio = FMath::Clamp(AirspeedMetersPerSecond / 85.0f, 0.0f, 1.0f);
    const float PropEfficiency =
        FMath::Lerp(1.0f, PropellerEfficiencyAtCruise, SpeedRatio);
    const float ThrustNewtons = MaximumThrustNewtons * Throttle * PropEfficiency;

    const FVector ThrustForce =
        GetActorForwardVector() * ThrustNewtons * SkyboundFlight::NewtonToUnrealForce;
    AircraftMesh->AddForce(ThrustForce);

    UpdateAerodynamics();
    UpdateFlightControls();
}

void ASkyboundAircraft::UpdateAerodynamics()
{
    const FVector AirVelocityMS = GetAirRelativeVelocityMS();
    const float SpeedMS = AirVelocityMS.Size();

    bStalled = false;
    AngleOfAttackDegrees = 0.0f;

    if (SpeedMS < SkyboundFlight::MinimumAirspeedMS)
    {
        return;
    }

    const FVector LocalAirVelocity =
        AircraftMesh->GetComponentTransform().InverseTransformVectorNoScale(AirVelocityMS);

    // Only use the normal forward-flight wing model while airflow comes from
    // ahead. Reverse flow is handled by drag, not by an inverted lift curve.
    if (LocalAirVelocity.X <= 0.0f)
    {
        const FVector ReverseDrag =
            -AirVelocityMS.GetSafeNormal() *
            DynamicPressurePascals * WingAreaSquareMeters *
            0.8f * SkyboundFlight::NewtonToUnrealForce;
        AircraftMesh->AddForce(ReverseDrag);
        return;
    }

    // Positive AoA means the nose is above the relative airflow.
    const float AlphaRad = FMath::Atan2(
        -LocalAirVelocity.Z,
        LocalAirVelocity.X
    );
    AngleOfAttackDegrees = AlphaRad / SkyboundFlight::DegToRad;

    const float ZeroLiftAngleRad =
        ZeroLiftAngleDegrees * SkyboundFlight::DegToRad;
    const float EffectiveAlpha = AlphaRad - ZeroLiftAngleRad;
    const float CriticalAlpha =
        CriticalAngleOfAttackDegrees * SkyboundFlight::DegToRad;
    const float AbsoluteAlpha = FMath::Abs(EffectiveAlpha);

    float LiftCoefficient = LiftCurveSlopePerRadian * EffectiveAlpha;

    // Smoothly reduce lift beyond critical AoA and increase drag. This gives
    // the aircraft a recognizable stall instead of unlimited linear lift.
    if (AbsoluteAlpha > CriticalAlpha)
    {
        bStalled = true;
        const float PeakLift = LiftCurveSlopePerRadian * CriticalAlpha;
        const float PostStallRatio =
            FMath::Exp(-(AbsoluteAlpha - CriticalAlpha) * 2.2f);
        LiftCoefficient = FMath::Sign(EffectiveAlpha) * PeakLift * PostStallRatio;
    }

    LiftCoefficient = FMath::Clamp(LiftCoefficient, -1.5f, 1.5f);

    const FVector AirflowDirection = AirVelocityMS.GetSafeNormal();
    const FVector LiftDirection =
        FVector::VectorPlaneProject(GetActorUpVector(), AirflowDirection).GetSafeNormal();

    if (!LiftDirection.IsNearlyZero())
    {
        const float LiftNewtons =
            DynamicPressurePascals * WingAreaSquareMeters * LiftCoefficient;

        AircraftMesh->AddForce(
            LiftDirection * LiftNewtons * SkyboundFlight::NewtonToUnrealForce
        );
    }

    const float DragCoefficient =
        ZeroLiftDragCoefficient +
        InducedDragFactor * FMath::Square(LiftCoefficient) +
        (bStalled ? StallDragIncrease : 0.0f);

    const float DragNewtons =
        DynamicPressurePascals * WingAreaSquareMeters * DragCoefficient;

    AircraftMesh->AddForce(
        -AirflowDirection * DragNewtons * SkyboundFlight::NewtonToUnrealForce
    );
}

void ASkyboundAircraft::UpdateFlightControls()
{
    const FVector AirVelocityMS = GetAirRelativeVelocityMS();
    const float SpeedMS = AirVelocityMS.Size();

    if (SpeedMS < SkyboundFlight::MinimumAirspeedMS)
    {
        return;
    }

    const FVector LocalAirVelocity =
        AircraftMesh->GetComponentTransform().InverseTransformVectorNoScale(AirVelocityMS);

    const float AlphaRad = FMath::Atan2(
        -LocalAirVelocity.Z,
        FMath::Max(0.1f, LocalAirVelocity.X)
    );

    const FVector AngularVelocityLocal =
        AircraftMesh->GetComponentTransform().InverseTransformVectorNoScale(
            AircraftMesh->GetPhysicsAngularVelocityInRadians()
        );

    const float DynamicPressure = DynamicPressurePascals;
    const float WingPitchScale =
        DynamicPressure * WingAreaSquareMeters * MeanChordMeters;
    const float WingRollYawScale =
        DynamicPressure * WingAreaSquareMeters * WingSpanMeters;

    // Positive W commands nose-up (negative local Y torque).
    // D commands right bank (negative local X torque); E commands right yaw.
    const float PitchControlNm =
        -PitchInput * ElevatorMomentCoefficient * WingPitchScale;
    const float PitchStabilityNm =
        AlphaRad * PitchStabilityCoefficient * WingPitchScale;
    const float PitchDampingNm =
        -AngularVelocityLocal.Y * PitchRateDamping;

    const float RollControlNm =
        -RollInput * AileronMomentCoefficient * WingRollYawScale;
    const float RollDampingNm =
        -AngularVelocityLocal.X * RollRateDamping;

    const float YawControlNm =
        YawInput * RudderMomentCoefficient * WingRollYawScale;
    const float YawDampingNm =
        -AngularVelocityLocal.Z * YawRateDamping;

    const FVector TorqueNm(
        RollControlNm + RollDampingNm,
        PitchControlNm + PitchStabilityNm + PitchDampingNm,
        YawControlNm + YawDampingNm
    );

    AircraftMesh->AddTorqueInRadians(
        TorqueNm * SkyboundFlight::NewtonMeterToUnrealTorque
    );
}
