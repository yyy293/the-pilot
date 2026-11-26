#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "SkyboundAircraft.generated.h"

class UStaticMeshComponent;

/**
 * Physics-driven aircraft pawn.
 *
 * Coordinate convention: Unreal +X forward, +Y right, +Z up.
 * Keyboard axes are configured in Config/DefaultInput.ini.
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

    // Pilot inputs. Values are clamped to [-1, 1], except throttle [0, 1].
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

    void UpdateAerodynamics(float DeltaTime);
    void UpdateFlightControls(float DeltaTime);

public:
    // Aircraft reference properties (SI units unless noted).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Aircraft", meta=(ClampMin="100.0"))
    float AircraftMassKg = 1111.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Engine", meta=(ClampMin="0.0"))
    float MaximumThrustNewtons = 2600.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Aerodynamics", meta=(ClampMin="1.0"))
    float WingAreaSquareMeters = 16.2f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Aerodynamics", meta=(ClampMin="0.1"))
    float LiftCurveSlopePerRadian = 5.2f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Aerodynamics", meta=(ClampMin="0.01"))
    float ZeroLiftDragCoefficient = 0.032f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Aerodynamics", meta=(ClampMin="0.0"))
    float InducedDragFactor = 0.055f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Aerodynamics", meta=(ClampMin="1.0", ClampMax="45.0"))
    float CriticalAngleOfAttackDegrees = 15.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Aerodynamics", meta=(ClampMin="0.0"))
    float ControlSurfaceLiftAuthority = 0.35f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Controls", meta=(ClampMin="0.01"))
    float ThrottleChangeRate = 0.35f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Controls", meta=(ClampMin="0.0"))
    float PitchControlMoment = 18000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Controls", meta=(ClampMin="0.0"))
    float RollControlMoment = 12000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Controls", meta=(ClampMin="0.0"))
    float YawControlMoment = 6500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Stability", meta=(ClampMin="0.0"))
    float PitchRateDamping = 4500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Stability", meta=(ClampMin="0.0"))
    float RollRateDamping = 5500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Stability", meta=(ClampMin="0.0"))
    float YawRateDamping = 3500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Environment", meta=(ClampMin="0.1"))
    float SeaLevelAirDensity = 1.225f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Flight|Environment", meta=(ClampMin="100.0"))
    float AtmosphericScaleHeightMeters = 8500.0f;

    UPROPERTY(BlueprintReadOnly, Category="Flight|Telemetry")
    float AirspeedMetersPerSecond = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category="Flight|Telemetry")
    float AirspeedKilometersPerHour = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category="Flight|Telemetry")
    float AngleOfAttackDegrees = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category="Flight|Telemetry")
    float VerticalSpeedMetersPerSecond = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category="Flight|Telemetry")
    bool bStalled = false;
};