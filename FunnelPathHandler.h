// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "AI/Navigation/NavigationTypes.h"
#include "FunnelPathHandler.generated.h"

class ARecastNavMesh;

/**
 *
 */
UCLASS()
class SMOOTHPATHPROJECT_API UFunnelPathHandler : public UObject
{
	GENERATED_BODY()

public:
	virtual bool SmoothPath(const FNavPathSharedPtr& NavPath);

	const TArray<FNavPathPoint>& GetFunnelPath() const { return FunnelPath; }

protected:
	virtual void AddPathPoint(FVector Location, FVector CurrentFunnelBase, int32 LocationEdgeIndex, const TArray<FNavigationPortalEdge>& PortalEdges, bool bLeft);
	
	TArray<FNavPathPoint> FunnelPath;
	TArray<FNavPathPoint> OriginalPath;
	const ARecastNavMesh* NavMesh;
};
