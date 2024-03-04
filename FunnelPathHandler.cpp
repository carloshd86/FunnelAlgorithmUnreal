// Fill out your copyright notice in the Description page of Project Settings.

#include "Navigation/FunnelPathHandler.h"
#include "Navigation/MetaNavMeshPath.h"

DECLARE_STATS_GROUP(TEXT("FunnelPathStatsGroup"), STATGROUP_FunnelPathStatsGroup, STATCAT_Advanced);
DECLARE_CYCLE_STAT(TEXT("FunnelPathFunction"), STAT_FunnelPathFunction, STATGROUP_FunnelPathStatsGroup);


float TriangleArea2D(FVector A, FVector B, FVector C)
{
	const float AX = B[0] - A[0];
	const float AY = B[1] - A[1];
	const float BX = C[0] - A[0];
	const float BY = C[1] - A[1];
	return BX * AY - AX * BY;
}

bool UFunnelPathHandler::SmoothPath(const FNavPathSharedPtr& NavPath)
{
	SCOPE_CYCLE_COUNTER(STAT_FunnelPathFunction);

	FunnelPath.Reset();
	OriginalPath.Reset();

	if (!NavPath.IsValid())
	{
		return false;
	}

	if (NavPath->GetPathPoints().Num() <= 2)
	{
		FunnelPath = NavPath->GetPathPoints();
		return true;
	}

	const auto NavMeshPathPtr = NavPath->CastPath<FNavMeshPath>();
	if (NavMeshPathPtr == nullptr)
	{
		return false;
	}

	const auto& NavMeshPath = *NavMeshPathPtr;

	const auto& CorridorEdges = NavMeshPath.GetPathCorridorEdges();
	if (CorridorEdges.Num() < 1)
	{
		return false;
	}

	OriginalPath = NavMeshPath.GetPathPoints();

	NavMesh = Cast<ARecastNavMesh>(NavMeshPath.GetNavigationDataUsed());

	const FVector StartLocation = NavMeshPath.GetStartLocation();
	const FVector EndLocation = NavMeshPath.GetEndLocation();

	FunnelPath.Add(StartLocation);
		
	int32 LeftEdgeIndex = 0;
	int32 RightEdgeIndex = 0;

	NavNodeRef LeftNodeRef = NavMeshPath.GetPathPoints()[0].NodeRef;
	NavNodeRef RightNodeRef = NavMeshPath.GetPathPoints()[0].NodeRef;

	const auto GetPositionOfNextCorridorWallVertex = [&LeftEdgeIndex, &RightEdgeIndex, &CorridorEdges, &EndLocation](bool bLeft)
	{
		int32 CurrentIndex = bLeft ? LeftEdgeIndex : RightEdgeIndex;
		const FVector PrevLocation = bLeft ? CorridorEdges[CurrentIndex].Left : CorridorEdges[CurrentIndex].Right;

		++CurrentIndex;
		while (CurrentIndex > CorridorEdges.Num())
		{
			const FVector NewLocation = bLeft ? CorridorEdges[CurrentIndex].Left : CorridorEdges[CurrentIndex].Right;
			if (NewLocation != PrevLocation)
			{
				return NewLocation;
			}
			++CurrentIndex;
		}

		return EndLocation;
	};

	FVector CurrentFunnelBase(StartLocation);
	FVector CurrentFunnelRight(StartLocation);
	FVector CurrentFunnelLeft(StartLocation);

	const int32 NumEdges = CorridorEdges.Num();
	for (int32 EdgeIndex = 0; EdgeIndex < NumEdges + 1; ++EdgeIndex)
	{
		FVector NewFunnelRight;
		FVector NewFunnelLeft;

		if (EdgeIndex < NumEdges)
		{
			const auto& Edge = CorridorEdges[EdgeIndex];

			NewFunnelRight = Edge.Right;
			NewFunnelLeft = Edge.Left;
		}
		else
		{
			NewFunnelRight = NewFunnelLeft = EndLocation;
		}

		// Check right vertex
		if (TriangleArea2D(CurrentFunnelBase, CurrentFunnelRight, NewFunnelRight) <= 0.f)
		{
			if (CurrentFunnelBase.Equals(CurrentFunnelRight) || TriangleArea2D(CurrentFunnelBase, CurrentFunnelLeft, NewFunnelRight) > 0.f)
			{
				// NewFunnelRight is inside the funnel, so make it the current right funnel vertex
				CurrentFunnelRight = NewFunnelRight;
				RightEdgeIndex = EdgeIndex;
				//RightNodeRef = Edge.ToRef;
			}
			else
			{
				// NewFunnelRight is beyond the left of the current funnel left
				// -> Make CurrentFunnelLeft the new funnel base and continue to the next loop

				if (CurrentFunnelLeft.Equals(EndLocation))
				{
					// If CurrentFunnelLeft is the end position, just exit as we will add the end position later
					break;
				}
				else
				{
					AddPathPoint(CurrentFunnelLeft, CurrentFunnelBase, LeftEdgeIndex, CorridorEdges, true);

					// Continue from the new funnel base corridor (the last valid left)
					EdgeIndex = RightEdgeIndex = LeftEdgeIndex;

					CurrentFunnelBase = CurrentFunnelRight = CurrentFunnelLeft = FunnelPath.Last().Location;
					continue;
				}
			}
		}

		// Check left vertex
		if (TriangleArea2D(CurrentFunnelBase, CurrentFunnelLeft, NewFunnelLeft) >= 0.f)
		{
			if (CurrentFunnelBase.Equals(CurrentFunnelLeft) || TriangleArea2D(CurrentFunnelBase, CurrentFunnelRight, NewFunnelLeft) < 0.f)
			{
				// NewFunnelLeft is inside the funnel, so make it the current left funnel vertex
				CurrentFunnelLeft = NewFunnelLeft;
				LeftEdgeIndex = EdgeIndex;
				//LeftNodeRef = Edge.ToRef;
			}
			else
			{
				// NewFunnelLeft is beyond the right of the current funnel right
				// -> Make CurrentFunnelRight the new funnel base and continue to the next loop

				if (CurrentFunnelRight.Equals(EndLocation))
				{
					// If CurrentFunnelRight is the end position, just exit as we will add the end position later
					break;
				}
				else
				{
					AddPathPoint(CurrentFunnelRight, CurrentFunnelBase, RightEdgeIndex, CorridorEdges, false);

					// Continue from the new funnel base corridor (the last valid right)
					EdgeIndex = LeftEdgeIndex = RightEdgeIndex;

					CurrentFunnelBase = CurrentFunnelRight = CurrentFunnelLeft = FunnelPath.Last().Location;
					continue;
				}
			}
		}
	}

	// Adding the end position at the end
	FunnelPath.Add(EndLocation);

	return true;
}

void UFunnelPathHandler::AddPathPoint(FVector Location, FVector CurrentFunnelBase, int32 LocationEdgeIndex, const TArray<FNavigationPortalEdge>& PortalEdges, bool bLeft)
{
	FunnelPath.Add(Location);
}