#include "GenerateFBX.h"
#include "PhysicsEngine/PhysicsAssetUtils.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/PhysicsConstraintTemplate.h"
#include "Engine/SkeletalMesh.h"
#include "Logging/LogMacros.h"
#include "AssetImportTask.h"
#include "Factories/FbxFactory.h"
#include "EditorAssetLibrary.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/Actor.h"
#include "Engine/Blueprint.h"
#include "Engine/SkeletalMesh.h"
#include "Components/SkeletalMeshComponent.h"

void UGenerateFBX::RegenerateBodies(UPhysicsAsset* PhysicsAsset, float MinBoneSize, EPhysAssetFitGeomType PrimitiveType)
{
    if (!ValidatePhysicsAsset(PhysicsAsset))
    {
        return;
    }

    // 現在のPhysicsAsset設定を更新
    PhysicsAsset->MinBoneSize = MinBoneSize;
    UE_LOG(LogTemp, Log, TEXT("Updated MinBoneSize to: %f"), MinBoneSize);

    PhysicsAsset->PhysicsType = PrimitiveType;
    UE_LOG(LogTemp, Log, TEXT("Updated PrimitiveType to: %d"), static_cast<int32>(PrimitiveType));

    // ボディを再生成
    const bool bRescale = false; // スケール変更をしない
    const bool bCreateNewAssets = true; // 新しいボディアセットを作成する

    int32 NumBodies = 0;

    // SkeletalMeshが正しい場合にのみ再生成を実行
    if (PhysicsAsset->GetPreviewMesh())
    {
        NumBodies = UPhysicsAssetUtils::CreateFromSkeletalMesh(
            PhysicsAsset,
            PhysicsAsset->GetPreviewMesh(),
            MinBoneSize,
            PrimitiveType,
            bRescale,
            bCreateNewAssets
        );
        UE_LOG(LogTemp, Log, TEXT("Successfully regenerated %d bodies in PhysicsAsset."), NumBodies);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("PhysicsAsset does not have a valid SkeletalMesh!"));
    }

    // 再生成後のアセットをマーク
    PhysicsAsset->MarkPackageDirty();
}

bool UGenerateFBX::ValidatePhysicsAsset(UPhysicsAsset* PhysicsAsset)
{
    if (!PhysicsAsset)
    {
        UE_LOG(LogTemp, Error, TEXT("PhysicsAsset is null!"));
        return false;
    }

    if (!PhysicsAsset->GetPreviewMesh())
    {
        UE_LOG(LogTemp, Error, TEXT("PhysicsAsset does not have a valid preview mesh!"));
        return false;
    }

    return true;
}

void UGenerateFBX::SetMinBoneSize(UPhysicsAsset* PhysicsAsset, float MinBoneSize)
{
    if (!ValidatePhysicsAsset(PhysicsAsset))
    {
        return;
    }

    PhysicsAsset->MinBoneSize = MinBoneSize;
    UE_LOG(LogTemp, Log, TEXT("MinBoneSize set to %f"), MinBoneSize);

    // アセットを変更したためDirtyマークを付与
    PhysicsAsset->MarkPackageDirty();
}

void UGenerateFBX::SetPrimitiveType(UPhysicsAsset* PhysicsAsset, EPhysAssetFitGeomType PrimitiveType)
{
    if (!ValidatePhysicsAsset(PhysicsAsset))
    {
        return;
    }

    PhysicsAsset->PhysicsType = PrimitiveType;
    UE_LOG(LogTemp, Log, TEXT("PrimitiveType set to %d"), static_cast<int32>(PrimitiveType));

    // アセットを変更したためDirtyマークを付与
    PhysicsAsset->MarkPackageDirty();
}


void UGenerateFBX::ImportFBXAndCreateBlueprint(const FString& FbxFilePath, const FString& BlueprintName, const FString& WheelBoneName)
{
    // Paths
    const FString DestinationPath = TEXT("/Game/ImportedFBX");
    FString FbxFileName = FPaths::GetBaseFilename(FbxFilePath);
    FString SkeletalMeshPath = FString::Printf(TEXT("%s/%s"), *DestinationPath, *FbxFileName);
    FString PhysicsAssetPath = FString::Printf(TEXT("%s/%s_PhysicsAsset"), *DestinationPath, *FbxFileName);
    FString BlueprintPath = FString::Printf(TEXT("/Game/%s"), *BlueprintName);

    // Delete existing assets
    if (UEditorAssetLibrary::DoesAssetExist(SkeletalMeshPath))
    {
        UEditorAssetLibrary::DeleteAsset(SkeletalMeshPath);
    }

    if (UEditorAssetLibrary::DoesAssetExist(PhysicsAssetPath))
    {
        UEditorAssetLibrary::DeleteAsset(PhysicsAssetPath);
    }

    if (UEditorAssetLibrary::DoesAssetExist(BlueprintPath))
    {
        UEditorAssetLibrary::DeleteAsset(BlueprintPath);
    }

    // Import FBX
    UAssetImportTask* ImportTask = NewObject<UAssetImportTask>();
    ImportTask->Filename = FbxFilePath;
    ImportTask->DestinationPath = DestinationPath;
    ImportTask->bReplaceExisting = true;
    ImportTask->bAutomated = true;

    UFbxFactory* ImportFactory = NewObject<UFbxFactory>();
    ImportTask->Options = ImportFactory;

    FAssetToolsModule& AssetToolsModule = FAssetToolsModule::GetModule();
    AssetToolsModule.Get().ImportAssetTasks({ ImportTask });

    // Load imported skeletal mesh
    USkeletalMesh* SkeletalMesh = Cast<USkeletalMesh>(UEditorAssetLibrary::LoadAsset(SkeletalMeshPath));
    if (!SkeletalMesh)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to load Skeletal Mesh from %s"), *SkeletalMeshPath);
        return;
    }

    // Create Blueprint
    UBlueprintFactory* BlueprintFactory = NewObject<UBlueprintFactory>();
    BlueprintFactory->ParentClass = AActor::StaticClass();

    UObject* NewBlueprint = AssetToolsModule.Get().CreateAsset(BlueprintName, TEXT("/Game"), UBlueprint::StaticClass(), BlueprintFactory);
    if (!NewBlueprint)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to create Blueprint %s"), *BlueprintName);
        return;
    }

    // Set Skeletal Mesh component
    UBlueprint* Blueprint = Cast<UBlueprint>(NewBlueprint);
    if (!Blueprint)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to cast Blueprint for %s"), *BlueprintName);
        return;
    }

    USkeletalMeshComponent* SkeletalMeshComponent = NewObject<USkeletalMeshComponent>(Blueprint->GeneratedClass);
    SkeletalMeshComponent->SetSkeletalMesh(SkeletalMesh);
    SkeletalMeshComponent->SetSimulatePhysics(true);
    SkeletalMeshComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

    // Save Blueprint
    UEditorAssetLibrary::SaveLoadedAsset(Blueprint);

    // Spawn Blueprint in the level
    FVector Location(0.f, 0.f, 0.f);
    FRotator Rotation(0.f, 0.f, 0.f);
    AActor* SpawnedActor = UGameplayStatics::BeginDeferredActorSpawnFromClass(
        GWorld, Blueprint->GeneratedClass, FTransform(Rotation, Location));

    if (SpawnedActor)
    {
        UGameplayStatics::FinishSpawningActor(SpawnedActor, FTransform(Rotation, Location));
        UE_LOG(LogTemp, Log, TEXT("Blueprint %s created and placed in the world."), *BlueprintName);
    }
}
