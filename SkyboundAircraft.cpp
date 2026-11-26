
#include "SkyboundAircraft.h"
#include "Components/StaticMeshComponent.h"
#include "GameFramework/PlayerController.h"

ASkyboundAircraft::ASkyboundAircraft()
{
    PrimaryActorTick.bCanEverTick = true;

    AircraftMesh = CreateDefaultSubobject<UStaticMeshComponent>(
        TEXT("AircraftMesh")
    );

    SetRootComponent(AircraftMesh);

    AircraftMesh->SetSimulatePhysics(true);
    AircraftMesh->SetEnableGravity(true);
    AircraftMesh->SetLinearDamping(0.02f);
    AircraftMesh->SetAngularDamping(0.4f);
}

void ASkyboundAircraft::BeginPlay()
{
    Super::BeginPlay();

    if (AircraftMesh)
    {
        AircraftMesh->WakeAllRigidBodies();
    }
}

void ASkyboundAircraft::SetThrottle(float Value)
{
    Throttle = FMath::Clamp(Throttle + Value, 0.0f, 1.0f);
}

void ASkyboundAircraft::SetPitch(float Value)
{
    PitchInput = Value;
}

void ASkyboundAircraft::SetRoll(float Value)
{
    RollInput = Value;
}

void ASkyboundAircraft::SetYaw(float Value)
{
    YawInput = Value;
}

void ASkyboundAircraft::SetupPlayerInputComponent(
    UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    PlayerInputComponent->BindAxis(
        TEXT("Pitch"), this, &ASkyboundAircraft::SetPitch);

    PlayerInputComponent->BindAxis(
        TEXT("Roll"), this, &ASkyboundAircraft::SetRoll);

    PlayerInputComponent->BindAxis(
        TEXT("Yaw"), this, &ASkyboundAircraft::SetYaw);

    PlayerInputComponent->BindAxis(
        TEXT("Throttle"), this, &ASkyboundAircraft::SetThrottle);
}

void ASkyboundAircraft::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (!AircraftMesh || !AircraftMesh->IsSimulatingPhysics())
        return;

    FVector Velocity = AircraftMesh->GetPhysicsLinearVelocity();

    // Convert Unreal centimetres per second to metres per second.
    FVector VelocityMS = Velocity / 100.0f;

    float Speed = VelocityMS.Size();

    AirspeedKmh = Speed * 3.6f;

    // Approximate atmospheric density based on altitude.
    float AltitudeMeters = GetActorLocation().Z / 100.0f;

    float Density = 1.225f * FMath::Exp(
        -FMath::Max(0.0f, AltitudeMeters) / 8500.0f
    );

    FVector Forward = GetActorForwardVector();
    FVector Right = GetActorRightVector();
    FVector Up = GetActorUpVector();

    // Engine thrust.
    FVector ThrustForce =
        Forward * MaxThrust * Throttle * 100.0f;

    AircraftMesh->AddForce(ThrustForce);

    if (Speed > 1.0f)
    {
        FVector Direction = VelocityMS.GetSafeNormal();

        FVector LocalVelocity =
            GetActorTransform().InverseTransformVector(Velocity);

        // Approximate angle of attack.
        float AngleOfAttack = FMath::Atan2(
            -LocalVelocity.Z,
            FMath::Max(1.0f, LocalVelocity.X)
        );

        // Lift coefficient with stall limits.
        float LiftCoefficient = FMath::Clamp(
            AngleOfAttack * LiftMultiplier,
            -1.2f,
            1.5f
        );

        if (Speed < StallSpeed)
        {
            LiftCoefficient *= Speed / StallSpeed;
        }

        float DynamicPressure =
            0.5f * Density * Speed * Speed;

        float Lift = DynamicPressure *
            WingArea * LiftCoefficient;

        FVector LiftDirection =
            FVector::CrossProduct(Direction, Right)
            .GetSafeNormal();

        if (FVector::DotProduct(LiftDirection, Up) < 0.0f)
        {
            LiftDirection *= -1.0f;
        }

        AircraftMesh->AddForce(
            LiftDirection * Lift * 100.0f
        );

        // Aerodynamic drag.
        float Drag = DynamicPressure *
            WingArea * DragCoefficient;

        AircraftMesh->AddForce(
            -Direction * Drag * 100.0f
        );
    }

    // Aircraft control forces.
    FVector Torque =
        Right * PitchInput * ControlTorque
        + Forward * -RollInput * ControlTorque
        + Up * YawInput * ControlTorque * 0.5f;

    AircraftMesh->AddTorqueInRadians(Torque);
}
