// Fill out your copyright notice in the Description page of Project Settings.
#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsAssetUtils.h"
#include "GenerateFBX.generated.h"

/**
 * A custom class to manipulate PhysicsAsset settings and regenerate bodies.
 */
UCLASS(Blueprintable, BlueprintType)
class ROBOTUE5APP_API UGenerateFBX : public UObject
{
    GENERATED_BODY()

public:
    /**
     * Regenerates bodies in the specified PhysicsAsset based on the given parameters.
     * 
     * @param PhysicsAsset The PhysicsAsset to modify and regenerate.
     * @param MinBoneSize The minimum size of bones to include.
     * @param PrimitiveType The type of primitive geometry to use (e.g., KSphere, KBox, KCapsule).
     */
    UFUNCTION(BlueprintCallable, Category = "PhysicsAsset")
    static void RegenerateBodies(UPhysicsAsset* PhysicsAsset, float MinBoneSize, EPhysAssetFitGeomType PrimitiveType);

    /**
     * Updates the MinBoneSize property of the PhysicsAsset.
     * 
     * @param PhysicsAsset The PhysicsAsset to modify.
     * @param MinBoneSize The new minimum bone size to set.
     */
    UFUNCTION(BlueprintCallable, Category = "PhysicsAsset")
    static void SetMinBoneSize(UPhysicsAsset* PhysicsAsset, float MinBoneSize);

    /**
     * Updates the PrimitiveType property of the PhysicsAsset.
     * 
     * @param PhysicsAsset The PhysicsAsset to modify.
     * @param PrimitiveType The new primitive type to set.
     */
    UFUNCTION(BlueprintCallable, Category = "PhysicsAsset")
    static void SetPrimitiveType(UPhysicsAsset* PhysicsAsset, EPhysAssetFitGeomType PrimitiveType);

    UFUNCTION(BlueprintCallable, Category = "FBX Import")
    static void ImportFBXAndCreateBlueprint(const FString& FbxFilePath, const FString& BlueprintName, const FString& WheelBoneName);

private:
    /**
     * Helper function to validate the PhysicsAsset before performing any operations.
     * 
     * @param PhysicsAsset The PhysicsAsset to validate.
     * @return True if the PhysicsAsset is valid; otherwise, false.
     */
    static bool ValidatePhysicsAsset(UPhysicsAsset* PhysicsAsset);
};
