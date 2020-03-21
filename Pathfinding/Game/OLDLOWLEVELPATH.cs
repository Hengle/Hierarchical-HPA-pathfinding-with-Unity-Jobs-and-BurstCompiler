/*private NativeList<float2> FindLowLevelPath( int jobIndex , int2 startGridPosition , int2 endGridPosition )
{
    #region SETUP DATA

    // Node sets
    NativeMinHeap openSet = new NativeMinHeap();
    openSet.Initialize( gridSize * gridSize , 0 , 100 );
    NativeArray<PathNode> nodes = new NativeArray<PathNode>( graphNodeArray.Length , Allocator.Temp );

    // Initialize nodes from ReadOnly grid array
    for ( int i = 0; i < graphNodeArray.Length; i++ )
    {
        PathNode node = new PathNode();
        node.gCost = int.MaxValue;
        node.parent = -1;
        node.graphArrayIndex = graphNodeArray[ i ].arrayIndex;
        node.isOpen = false;
        node.isClosed = false;

        nodes[ i ] = node;
    }

    int endNodeIndex = endGridPosition.x + endGridPosition.y * gridSize;

    // Initialize the starting node 
    PathNode startNode = nodes[ startGridPosition.x + startGridPosition.y * gridSize ];
    startNode.gCost = 0;
    startNode.hCost = ManhattenDistance( new int2( graphNodeArray[ startNode.graphArrayIndex ].col , graphNodeArray[ startNode.graphArrayIndex ].row ) , endGridPosition );
    startNode.fCost = startNode.hCost;
    startNode.isOpen = true;
    nodes[ startNode.graphArrayIndex ] = startNode;

    openSet.Enqueue( startNode.graphArrayIndex , startNode.fCost );

    #endregion
    #region FIND PATH

    while ( openSet.Length > 0 )
    {
        int currentNodeIndex = openSet.DequeueMin( nodes );

        // Break if we reached our goal
        if ( currentNodeIndex == endNodeIndex )
            break;

        GraphNode gridNode = graphNodeArray[ currentNodeIndex ];
        PathNode node = nodes[ currentNodeIndex ];
        node.isOpen = false;
        node.isClosed = true;
        nodes[ currentNodeIndex ] = node;

        var neighbours = neighbourMap.GetValuesForKey( currentNodeIndex );

        do
        {
            int neighbour = neighbours.Current;

            int2 neighbourPosition = new int2( graphNodeArray[ neighbour ].col , graphNodeArray[ neighbour ].row );
            int neighbourNodeIndex = neighbourPosition.x + neighbourPosition.y * gridSize;

            if ( nodes[ neighbourNodeIndex ].isClosed )
                continue;

            int tentativeCost =
                nodes[ currentNodeIndex ].gCost +
                ManhattenDistance(
                    new int2( gridNode.col , gridNode.row ) ,
                    new int2( graphNodeArray[ neighbourNodeIndex ].col , graphNodeArray[ neighbourNodeIndex ].row ) ) +
                graphNodeArray[ neighbourNodeIndex ].movementPenalty;

            PathNode neighbourNode = nodes[ neighbourNodeIndex ];
            int hCost = ManhattenDistance(
                new int2( neighbourPosition.x , neighbourPosition.y ) , endGridPosition );

            if ( tentativeCost + 1 < neighbourNode.gCost )
            {
                neighbourNode.parent = currentNodeIndex;
                neighbourNode.hCost = hCost;
                neighbourNode.gCost = tentativeCost;
                neighbourNode.fCost = neighbourNode.gCost + neighbourNode.hCost;

                if ( !neighbourNode.isOpen )
                {
                    neighbourNode.isOpen = true;
                    openSet.Enqueue( neighbourNodeIndex , neighbourNode.fCost );
                }

                nodes[ neighbourNodeIndex ] = neighbourNode;
            }
        }
        while ( neighbours.MoveNext() );
    }

    #endregion
    #region TRACE PATH

    PathNode endNode = nodes[ endNodeIndex ];
    NativeList<int2> aStarPath = new NativeList<int2>( Allocator.Temp );
    NativeList<float2> waypoints = new NativeList<float2>( Allocator.Temp );

    if ( endNode.parent == -1 )
    {
        nodes.Dispose();
        aStarPath.Dispose();
        openSet.Dispose();
        return waypoints;
    }
    else
    {
        waypoints.Add( endWorldPositions[ jobIndex ] );
        aStarPath.Add( endGridPosition );

        while ( endNode.parent != -1 )
        {
            int parentIndex = endNode.parent;

            aStarPath.Add( new int2(
                graphNodeArray[ parentIndex ].col ,
                graphNodeArray[ parentIndex ].row ) );

            /*waypoints.Add( new float2(
                gridVertexNodeArray[ parentIndex ].x * gridCellSize , //+ cellSize / 2 ,
                gridVertexNodeArray[ parentIndex ].y * gridCellSize //+ cellSize / 2
            ) );*/

/* endNode = nodes[ parentIndex ];
}
}*/

//nodes.Dispose();
//openSet.Dispose();

//return waypoints;

/*#endregion
#region SMOOTH PATH

if ( aStarPath.Length > 2 ) // If its less than or equal 2 theres no need to smooth the path
{
    int fromIndex = 0;
    bool foundPath = false;

    while ( !foundPath )
    {
        int currentIndex = fromIndex + 2; // Because the next index is always going to be in line of sight

        if ( currentIndex > aStarPath.Length - 1 )
            break;

        while ( true )
        {
            int stopIndex = currentIndex - 1;
            int pathNodeArrayIndex = aStarPath[ stopIndex ].x + aStarPath[ stopIndex ].y * gridSize;

            int2 start = aStarPath[ fromIndex ];
            int2 end = aStarPath[ currentIndex ];

            if ( !LOS( start.x , start.y , end.x , end.y ) )
            {
                float2 worldPosition = new float2(
                    ( graphNodeArray[ pathNodeArrayIndex ].col * gridCellSize ) ,
                    ( graphNodeArray[ pathNodeArrayIndex ].row * gridCellSize )
                );
                waypoints.Add( worldPosition );
                fromIndex = stopIndex;
                break;
            }
            else
            {
                if ( currentIndex >= aStarPath.Length - 1 )
                {
                    foundPath = true;
                    break;
                }
                currentIndex++;
            }
        }
    }
}

#endregion
#region RETURN

aStarPath.Dispose();
return waypoints;

#endregion
}*/

/*private NativeList<float2> FindLowLevelPathInCluster( int jobIndex , int2 startPosition , int2 endPosition , int2 clusterPosition )
 {
     #region SETUP DATA

     // Node sets
     NativeMinHeap openSet = new NativeMinHeap();
     openSet.Initialize( gridClusterSize * gridClusterSize , 0 , gridClusterSize * gridClusterSize + 1 );
     NativeArray<PathNode> pathNodes = new NativeArray<PathNode>( gridClusterSize * gridClusterSize , Allocator.Temp );

     // Initialize nodes from ReadOnly grid array
     for ( int localRow = 0; localRow < gridClusterSize; localRow++ )
     {
         for ( int localCol = 0; localCol < gridClusterSize; localCol++ )
         {
             PathNode node = new PathNode();
             node.gCost = int.MaxValue;
             node.parent = -1;

             int localArrayIndex = localCol + localRow * gridClusterSize;

             int graphCol = localCol + gridClusterSize * clusterPosition.x;
             int graphRow = localRow + gridClusterSize * clusterPosition.y;
             int graphArrayIndex = graphCol + graphRow * gridNumCells;

             node.localArrayIndex = localArrayIndex;
             node.graphArrayIndex = graphArrayIndex;

             node.isOpen = false;
             node.isClosed = false;

             pathNodes[ localArrayIndex ] = node;
         }
     }

     // Get and cache the start and end pathNodeIndices
     int endNodeX = endPosition.x - clusterPosition.x * gridClusterSize;
     int endNodeY = endPosition.y - clusterPosition.y * gridClusterSize;
     int localEndNodeIndex = endNodeX + endNodeY * gridClusterSize;
     int startNodeX = startPosition.x - clusterPosition.x * gridClusterSize;
     int startNodeY = startPosition.y - clusterPosition.y * gridClusterSize;
     int localStartNodeIndex = startNodeX + startNodeY * gridClusterSize;

     // Initialize the starting pathNode
     PathNode startNode = pathNodes[ localStartNodeIndex ];
     startNode.gCost = 0;
     startNode.hCost = ManhattenDistance( 
         new int2( 
             graphNodeArray[ startNode.graphArrayIndex ].col ,
             graphNodeArray[ startNode.graphArrayIndex ].row ) , 
         endPosition );
     startNode.fCost = startNode.hCost;
     startNode.isOpen = true;
     pathNodes[ startNode.localArrayIndex ] = startNode;

     // Add the start pathNode to the openSet
     //openSet.Add( startNode.localArrayIndex );
     openSet.Enqueue( startNode.localArrayIndex , startNode.fCost );

     #endregion
     #region SEARCH GRAPH

     while ( openSet.Length > 0 )
     {
         // Cache the pathNodeIndex we are working with during this iteration
         int currentPathNodeIndex = openSet.DequeueMin1( pathNodes );

         // Break if we reached our goal
         if ( currentPathNodeIndex == localEndNodeIndex )
             break;

         PathNode pathNode = pathNodes[ currentPathNodeIndex ];
         pathNode.isOpen = false;
         pathNode.isClosed = true;
         pathNodes[ currentPathNodeIndex ] = pathNode;

         // Get and cache the graphNode for convenience
         int graphIndex = pathNodes[ currentPathNodeIndex ].graphArrayIndex;
         Node graphNode = graphNodeArray[ graphIndex ];

         // Iterate over the neighbours of the current pathNode
         int keyIndex = pathNodes[ currentPathNodeIndex ].graphArrayIndex;
         int2 neighbourKey = graphNodeNeighbourKeyArray[ keyIndex ];
         for ( int i = neighbourKey.x; i < neighbourKey.y; i++ )
         {
             int neighbourGraphArrayIndex = grahNodeNeighbourArray[ i ];

             int2 neighbourGridPosition = new int2(
                 graphNodeArray[ neighbourGraphArrayIndex ].col ,
                 graphNodeArray[ neighbourGraphArrayIndex ].row );

             // Get the local neighbour index
             int localNeighbourCol = neighbourGridPosition.x - clusterPosition.x * gridClusterSize;
             int localNeighbourRow = neighbourGridPosition.y - clusterPosition.y * gridClusterSize;
             int neighbourLocalArrayIndex = localNeighbourCol + localNeighbourRow * gridClusterSize;

             // Skip if its closed (already searched)
             if ( pathNodes[ neighbourLocalArrayIndex ].isClosed )
                 continue;

             // Calculate the cost to move from current node to neighbour node
             int distanceCost = ManhattenDistance(
                 new int2( graphNode.col , graphNode.row ) ,
                 new int2( graphNodeArray[ pathNodes[ neighbourLocalArrayIndex ].graphArrayIndex ].col ,
                           graphNodeArray[ pathNodes[ neighbourLocalArrayIndex ].graphArrayIndex ].row ) );
             int movementPenalty =
                 graphNodeArray[ pathNodes[ neighbourLocalArrayIndex ].graphArrayIndex ].movementPenalty;
             int tentativeCost =
                 pathNodes[ currentPathNodeIndex ].gCost + distanceCost + movementPenalty + heuristicBias;

             PathNode pathNeighbourNode = pathNodes[ neighbourLocalArrayIndex ];
             int hCost = ManhattenDistance(
                 new int2( neighbourGridPosition.x , neighbourGridPosition.y ) , endPosition );

             if ( tentativeCost < pathNeighbourNode.gCost )
             {
                 pathNeighbourNode.parent = currentPathNodeIndex;
                 pathNeighbourNode.hCost = hCost;
                 pathNeighbourNode.gCost = tentativeCost;
                 pathNeighbourNode.fCost = pathNeighbourNode.gCost + pathNeighbourNode.hCost;

                 if ( !pathNeighbourNode.isOpen )
                 {
                     pathNeighbourNode.isOpen = true;
                     openSet.Enqueue( neighbourLocalArrayIndex , pathNeighbourNode.fCost );
                 }

                 pathNodes[ neighbourLocalArrayIndex ] = pathNeighbourNode;
             }
         }
     }

     #endregion
     #region TRACE PATH

     PathNode endPathNode = pathNodes[ localEndNodeIndex ];
     NativeList<int2> aStarPath = new NativeList<int2>( Allocator.Temp );
     NativeList<float2> waypoints = new NativeList<float2>( Allocator.Temp );

     if ( endPathNode.parent == -1 )
     {
         pathNodes.Dispose();
         aStarPath.Dispose();
         openSet.Dispose();
         return waypoints;
     }
     else
     {
         waypoints.Add( endWorldPositions[ jobIndex ] );
         aStarPath.Add( endPosition );

         while ( endPathNode.parent != -1 )
         {
             int parentIndex = endPathNode.parent;

             aStarPath.Add( new int2(
                 graphNodeArray[ pathNodes[ parentIndex ].graphArrayIndex ].col ,
                 graphNodeArray[ pathNodes[ parentIndex ].graphArrayIndex ].row ) );

             endPathNode = pathNodes[ parentIndex ];
         }
     }

     pathNodes.Dispose();
     openSet.Dispose();

     #endregion
     #region SMOOTH PATH

     if ( aStarPath.Length > 2 ) // If its less than or equal 2 theres no need to smooth the path
     {
         int fromIndex = 0;
         bool foundPath = false;

         while ( !foundPath )
         {
             int currentIndex = fromIndex + 2; // Because the next index is always going to be in line of sight

             if ( currentIndex > aStarPath.Length - 1 )
                 break;

             while ( true )
             {
                 int stopIndex = currentIndex - 1;
                 int graphNodeArrayIndex = aStarPath[ stopIndex ].x + aStarPath[ stopIndex ].y * gridNumCells;

                 int2 start = aStarPath[ fromIndex ];
                 int2 end = aStarPath[ currentIndex ];

                 if ( !LOS( start.x , start.y , end.x , end.y ) )
                 {
                     float2 worldPosition = new float2(
                         ( graphNodeArray[ graphNodeArrayIndex ].col * gridCellSize ) ,
                         ( graphNodeArray[ graphNodeArrayIndex ].row * gridCellSize )
                     );
                     waypoints.Add( worldPosition );
                     fromIndex = stopIndex;
                     break;
                 }
                 else
                 {
                     if ( currentIndex >= aStarPath.Length - 1 )
                     {
                         foundPath = true;
                         break;
                     }
                     currentIndex++;
                 }
             }
         }
     }

     #endregion
     #region RETURN

     aStarPath.Dispose();
     return waypoints;

     #endregion
 }*/