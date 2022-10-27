// Copyright Epic Games, Inc. All Rights Reserved.

#include "Navigation/LoSPathFollowingComponent.h"
#include "AbstractNavData.h"
#include "NavMesh/NavMeshPath.h"
#include "NavMesh/RecastNavMesh.h"


int32 ULoSPathFollowingComponent::DetermineCurrentTargetPathPoint(int32 StartIndex)
{
	if (Path->GetPathPoints()[StartIndex].CustomLinkId 
		|| Path->GetPathPoints()[StartIndex + 1].CustomLinkId)
	{
		return StartIndex + 1;
	}

#if WITH_RECAST
	FNavMeshNodeFlags Flags0(Path->GetPathPoints()[StartIndex].Flags);
	FNavMeshNodeFlags Flags1(Path->GetPathPoints()[StartIndex + 1].Flags);

	const bool bOffMesh0 = (Flags0.PathFlags & RECAST_STRAIGHTPATH_OFFMESH_CONNECTION) != 0;
	const bool bOffMesh1 = (Flags1.PathFlags & RECAST_STRAIGHTPATH_OFFMESH_CONNECTION) != 0;

	if ((Flags0.Area != Flags1.Area) || (bOffMesh0 && bOffMesh1))
	{
		return StartIndex + 1;
	}
#endif

	return OptimizeSegmentVisibility(StartIndex);
}

int32 ULoSPathFollowingComponent::OptimizeSegmentVisibility(const int32 StartIndex)
{
	const AController* MyController = Cast<AController>(GetOwner());
	const ANavigationData* NavData = Path.IsValid() ? Path->GetNavigationDataUsed() : NULL;
	const APawn* MyPawn = MyController ? MyController->GetPawn() : nullptr;
	if (NavData == nullptr || MyController == nullptr || MyPawn == nullptr)
	{
		return StartIndex + 1;
	}

	const bool bIsDirect = (Path->CastPath<FAbstractNavigationPath>() != NULL);
	if (bIsDirect)
	{
		// can't optimize anything without real corridor
		return StartIndex + 1;
	}

	float DummyRadius = 0.f;
	float PawnHalfHeight = 0.f;
	MyPawn->GetSimpleCollisionCylinder(DummyRadius, PawnHalfHeight);
	const FVector PawnLocation = MyPawn->GetActorLocation();

	FSharedNavQueryFilter QueryFilter = Path->GetFilter()->GetCopy();
#if WITH_RECAST
	// priming QueryFilter this way will make sure we LoS-pick only the points of the same area type. 
	const uint8 StartArea = FNavMeshNodeFlags(Path->GetPathPoints()[StartIndex].Flags).Area;
	TArray<float> CostArray;
	CostArray.Init(RECAST_UNWALKABLE_POLY_COST, RECAST_MAX_AREAS);
	CostArray[StartArea] = 0;
	QueryFilter->SetAllAreaCosts(CostArray);
#endif

	const int32 MaxPoints = Path->GetPathPoints().Num();
	int32 Index = StartIndex + 2;
	Path->ShortcutNodeRefs.Reset();

	for (; Index <= MaxPoints; ++Index)
	{
		if (!Path->GetPathPoints().IsValidIndex(Index))
		{
			break;
		}

		const FNavPathPoint& PathPt1 = Path->GetPathPoints()[Index];
		const FVector PathPt1Location = *Path->GetPathPointLocation(Index);
		const FVector AdjustedDestination = PathPt1Location + FVector(0.0f, 0.0f, PawnHalfHeight);

		FVector HitLocation;
#if WITH_RECAST
		ARecastNavMesh::FRaycastResult RaycastResult;
		const bool RaycastHitResult = ARecastNavMesh::NavMeshRaycast(NavData, PawnLocation, AdjustedDestination, HitLocation, QueryFilter, MyController, RaycastResult);
		if (Path.IsValid())
		{
			Path->ShortcutNodeRefs.Reserve(RaycastResult.CorridorPolysCount);
			Path->ShortcutNodeRefs.SetNumUninitialized(RaycastResult.CorridorPolysCount);

			FPlatformMemory::Memcpy(Path->ShortcutNodeRefs.GetData(), RaycastResult.CorridorPolys, RaycastResult.CorridorPolysCount * sizeof(NavNodeRef));
		}
#else
		const bool RaycastHitResult = NavData->Raycast(PawnLocation, AdjustedDestination, HitLocation, QueryFilter);
#endif
		if (RaycastHitResult)
		{
			break;
		}

#if WITH_RECAST
		/** don't move to next point if we are changing area, we are at off mesh connection or it's a custom nav link*/
		if (FNavMeshNodeFlags(Path->GetPathPoints()[StartIndex].Flags).Area != FNavMeshNodeFlags(PathPt1.Flags).Area ||
			FNavMeshNodeFlags(PathPt1.Flags).PathFlags & RECAST_STRAIGHTPATH_OFFMESH_CONNECTION || 
			PathPt1.CustomLinkId)
		{
			return Index;
		}
#endif
	}

	return Index-1;
}