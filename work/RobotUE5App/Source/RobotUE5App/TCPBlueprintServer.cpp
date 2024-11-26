#include "TCPBlueprintServer.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Networking.h"

void UTCPBlueprintServer::StartServer(int32 Port)
{
    // Create a socket for the server
    ServerSocket = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateSocket(NAME_Stream, TEXT("TCPServer"), false);

    // Bind to the port
    FIPv4Endpoint Endpoint(FIPv4Address::Any, Port);
    if (!ServerSocket->Bind(*Endpoint.ToInternetAddr()))
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to bind to port %d"), Port);
        return;
    }

    // Start listening
    if (!ServerSocket->Listen(8)) // Max 8 connections
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to start listening on port %d"), Port);
        return;
    }

    UE_LOG(LogTemp, Log, TEXT("TCP Server started on port %d"), Port);

    // Start a separate thread to handle incoming connections
    ListenForConnections();
}

void UTCPBlueprintServer::StopServer()
{
    if (ClientSocket)
    {
        ClientSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ClientSocket);
    }
    if (ServerSocket)
    {
        ServerSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ServerSocket);
    }

    UE_LOG(LogTemp, Log, TEXT("TCP Server stopped"));
}

void UTCPBlueprintServer::ListenForConnections()
{
    while (true)
    {
        // Accept incoming connection
        ClientSocket = ServerSocket->Accept(TEXT("TCPClient"));
        if (ClientSocket)
        {
            UE_LOG(LogTemp, Log, TEXT("Client connected"));
            HandleClientConnection();
        }
    }
}

void UTCPBlueprintServer::HandleClientConnection()
{
    FString ReceivedMessage = ReceiveData(ClientSocket);
    if (ReceivedMessage.IsEmpty())
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to receive data from client"));
        return;
    }

    // Parse the received message
    TArray<FString> ParsedData;
    ReceivedMessage.ParseIntoArray(ParsedData, TEXT(";"), true);

    if (ParsedData.Num() < 3)
    {
        UE_LOG(LogTemp, Error, TEXT("Invalid data format. Expected: <FBXFilePath>;<BlueprintName>;<WheelBoneName>"));
        return;
    }

    FString FbxFilePath = ParsedData[0];
    FString BlueprintName = ParsedData[1];
    FString WheelBoneName = ParsedData[2];

    // Call the Blueprint importer function
    UGenerateFBX::ImportFBXAndCreateBlueprint(FbxFilePath, BlueprintName, WheelBoneName);

    UE_LOG(LogTemp, Log, TEXT("Processed message: %s"), *ReceivedMessage);
}

FString UTCPBlueprintServer::ReceiveData(FSocket* Socket)
{
    TArray<uint8> ReceivedData;
    uint32 Size;

    // Check if there is pending data
    while (Socket->HasPendingData(Size))
    {
        ReceivedData.SetNumUninitialized(FMath::Min(Size, 65507u));

        int32 Read = 0;
        Socket->Recv(ReceivedData.GetData(), ReceivedData.Num(), Read);
    }

    // Convert received data to string
    FString ReceivedString = FString(ANSI_TO_TCHAR(reinterpret_cast<const char*>(ReceivedData.GetData())));
    return ReceivedString;
}

