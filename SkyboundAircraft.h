#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "SkyboundAircraft.generated.h"

class UStaticMesh;
class UStaticMeshComponent;

/**
 * Physics-driven fixed-wing aircraft pawn.
 *
 * Coordinate convention for the aircraft mesh:
 *   +X = forward, +Y = right wing, +Z = up.
 *
 * The imported aircraft mesh must have simple collision suitable for
 * simulation. Tune its pivot/origin in Blender so it is near the aircraft CG.
 */
UCLASS()
class SKYBOUND_API ASkyboundAircraft : public APawn
{
    GENERATED_BODY()

public:
    ASkyboundAircraft();

    virtual void Tick(float DeltaTime) override;
    virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

protected:
    virtual void BeginPlay() override;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Aircraft")
    TObjectPtr<UStaticMeshComponent> AircraftMesh;

    // Assign the imported Cessna Static Mesh in the Blueprint defaults.
    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category="Aircraft|Model")
    TObjectPtr<UStaticMesh> AircraftModel;

    UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category="Flight|Controls")
    float Throttle = 0.0f;

    float ThrottleAxis = 0.0f;
    float PitchInput = 0.0f;
    float RollInput = 0.0f;
    float YawInput = 0.0f;

    void SetThrottleAxis(float Value);
    void SetPitch(float Value);
    void SetRoll(float Value);
    void SetYaw(float Value);

    void UpdateAerodynamics();
    void UpdateFlightControls();

    FVector GetAirRelativeVelocityMS() const;
    float GetAirDensityKgPerM3() const;
    float GetDynamicPressurePa() const;

public:
    // Mass and engine
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Aircraft", meta=(ClampMin="100.0"))
    float AircraftMassKg = 1111.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Engine", meta=(ClampMin="0.0"))
    float MaximumThrustNewtons = 2600.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Engine", meta=(ClampMin="0.0", ClampMax="1.0"))
    float PropellerEfficiencyAtCruise = 0.82f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Controls", meta=(ClampMin="0.01"))
    float ThrottleChangeRate = 0.35f;

    // Wing / aerodynamic coefficients
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Aerodynamics", meta=(ClampMin="1.0"))
    float WingAreaSquareMeters = 16.2f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Aerodynamics", meta=(ClampMin="0.1"))
    float WingSpanMeters = 11.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Aerodynamics", meta=(ClampMin="0.1"))
    float MeanChordMeters = 1.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Aerodynamics")
    float ZeroLiftAngleDegrees = -2.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Aerodynamics", meta=(ClampMin="0.1"))
    float LiftCurveSlopePerRadian = 5.2f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Aerodynamics", meta=(ClampMin="0.01"))
    float ZeroLiftDragCoefficient = 0.032f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Aerodynamics", meta=(ClampMin="0.0"))
    float InducedDragFactor = 0.055f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Aerodynamics", meta=(ClampMin="1.0", ClampMax="45.0"))
    float CriticalAngleOfAttackDegrees = 15.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Aerodynamics", meta=(ClampMin="0.0"))
    float StallDragIncrease = 0.22f;

    // Control effectiveness is scaled by dynamic pressure, so controls become
    // less effective at low airspeed and stronger as the aircraft accelerates.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Controls", meta=(ClampMin="0.0"))
    float ElevatorMomentCoefficient = 0.42f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Controls", meta=(ClampMin="0.0"))
    float AileronMomentCoefficient = 0.075f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Controls", meta=(ClampMin="0.0"))
    float RudderMomentCoefficient = 0.055f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Stability", meta=(ClampMin="0.0"))
    float PitchStabilityCoefficient = 0.035f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Stability", meta=(ClampMin="0.0"))
    float PitchRateDamping = 9000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Stability", meta=(ClampMin="0.0"))
    float RollRateDamping = 7000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Stability", meta=(ClampMin="0.0"))
    float YawRateDamping = 5500.0f;

    // Environment. World Z=0 is treated as sea level for this prototype.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Environment", meta=(ClampMin="0.1"))
    float SeaLevelAirDensity = 1.225f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Environment", meta=(ClampMin="100.0"))
    float AtmosphericScaleHeightMeters = 8500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Environment")
    FVector WindVelocityMetersPerSecond = FVector::ZeroVector;

    // Telemetry for a future cockpit HUD.
    UPROPERTY(BlueprintReadOnly, Category="Flight|Telemetry")
    float AirspeedMetersPerSecond = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category="Flight|Telemetry")
    float AirspeedKilometersPerHour = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category="Flight|Telemetry")
    float AngleOfAttackDegrees = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category="Flight|Telemetry")
    float VerticalSpeedMetersPerSecond = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category="Flight|Telemetry")
    float DynamicPressurePascals = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category="Flight|Telemetry")
    bool bStalled = false;
};
