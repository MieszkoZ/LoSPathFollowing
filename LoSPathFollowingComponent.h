// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Navigation/PathFollowingComponent.h"
#include "LoSPathFollowingComponent.generated.h"


UCLASS()
class ULoSPathFollowingComponent : public UPathFollowingComponent
{
	GENERATED_BODY()
protected:
	virtual int32 DetermineCurrentTargetPathPoint(int32 StartIndex) override;

	int32 OptimizeSegmentVisibility(const int32 StartIndex);
};
