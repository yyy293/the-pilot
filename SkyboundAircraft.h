
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "SkyboundAircraft.generated.h"

UCLASS()
class SKYBOUND_API ASkyboundAircraft : public APawn
{
    GENERATED_BODY()

public:
    ASkyboundAircraft();

protected:
    virtual void BeginPlay() override;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    TObjectPtr<UStaticMeshComponent> AircraftMesh;

    float Throttle = 0.0f;
    float PitchInput = 0.0f;
    float RollInput = 0.0f;
    float YawInput = 0.0f;

    void SetThrottle(float Value);
    void SetPitch(float Value);
    void SetRoll(float Value);
    void SetYaw(float Value);

public:
    virtual void Tick(float DeltaTime) override;

    virtual void SetupPlayerInputComponent(
        UInputComponent* PlayerInputComponent
    ) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Aircraft")
    float MaxThrust = 90000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Aircraft")
    float WingArea = 28.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Aircraft")
    float LiftMultiplier = 5.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Aircraft")
    float DragCoefficient = 0.035f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Aircraft")
    float ControlTorque = 2500000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Aircraft")
    float StallSpeed = 45.0f;

    UPROPERTY(BlueprintReadOnly, Category="Aircraft")
    float AirspeedKmh = 0.0f;
};
