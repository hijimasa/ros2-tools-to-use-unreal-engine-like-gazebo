#pragma once

#include "CoreMinimal.h"
#include "Networking.h"
#include "GenerateFBX.h"
#include "TCPBlueprintServer.generated.h"

UCLASS()
class ROBOTUE5APP_API UTCPBlueprintServer : public UObject
{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable, Category = "TCP Server")
    void StartServer(int32 Port);

    UFUNCTION(BlueprintCallable, Category = "TCP Server")
    void StopServer();

private:
    FSocket* ServerSocket;
    FSocket* ClientSocket;

    void ListenForConnections();
    void HandleClientConnection();
    FString ReceiveData(FSocket* Socket);
};
