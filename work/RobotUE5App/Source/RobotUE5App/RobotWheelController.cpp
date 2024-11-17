// Fill out your copyright notice in the Description page of Project Settings.


#include "RobotWheelController.h"


// Sets default values
ARobotWheelController::ARobotWheelController()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ARobotWheelController::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void ARobotWheelController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

