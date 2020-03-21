/*private void AssignNodeNeighbours()
{
    // Temporary list to convert to native array later
    NativeList<int> nodeNeighbourList = new NativeList<int>( Allocator.Temp );
    nodeNeighbourLists = new NativeArray<int2>( gridCapacity , Allocator.Persistent );

    int currentListIndex = 0;

    for ( int i = 0; i < gridCapacity; i++ )
    {
        // if the node is walkable
        if ( nodeWalkables[ i ] == 0 )
        {
            int2 nodePos = nodePositions[ i ];
            // The cluster boundaries essentially
            int clusterX = ( nodePos.x / clusterSize ) * clusterSize;
            int clusterY = ( nodePos.y / clusterSize ) * clusterSize;
            int2 minNodePos = new int2( clusterX , clusterY );
            int2 maxNodePos = new int2( minNodePos.x + clusterSize , minNodePos.y + clusterSize );

            // Loop a square around the node, omitting the node itself
            for ( int y = nodePos.y - 1; y < nodePos.y + 2; y++ )
            {
                for ( int x = nodePos.x - 1; x < nodePos.x + 2; x++ )
                {
                    if ( y == nodePos.y && x == nodePos.x )
                        continue;

                    // If the node is in bounderies and is walkable, add neighbour index to temp list
                    if ( x >= minNodePos.x && x < maxNodePos.x && y >= minNodePos.y && y < maxNodePos.y && nodeWalkables[ x + y * numCellsAcross ] == 0 )
                    {
                        nodeNeighbourList.Add( x + y * numCellsAcross );
                    }
                }
            }
        }

        // Add the key to the array and set update the current list index for the next path
        nodeNeighbourLists[ i ] = new int2( currentListIndex , nodeNeighbourList.Length );
        currentListIndex = nodeNeighbourList.Length;
    }

    // Finally convert the list to the native array and dispose of the temporary list
    int[] tempNeighbourArray = nodeNeighbourList.ToArray();
    nodeNeighbours = new NativeArray<int>( tempNeighbourArray , Allocator.Persistent );
    nodeNeighbourList.Dispose();
}*/

//public NativeArray<int2> nodeNeighbourLists; // same length
//public NativeArray<int> nodeNeighbours;

// GRAPH JOB

/*int2 neighbourKey = nodeNeighbourKeyArray[ graphIndexArray[ currentNodeIndex ] ];
for ( int i = neighbourKey.x; i < neighbourKey.y; i++ )
{
    int graphNeighbour = nodeNeighbourArray[ i ];
    int2 graphNeighbourPos = nodePositionArray[ graphNeighbour ];

    // Get the local neighbour index
    int neighbourCol = graphNeighbourPos.x - clusterPosition.x * clusterSize;
    int neighbourRow = graphNeighbourPos.y - clusterPosition.y * clusterSize;
    int localNeighbourIndex = neighbourCol + neighbourRow * clusterSize;

    // Skip if its closed (already searched)
    if ( closedArray[ localNeighbourIndex ] )
        continue;

    // Calculate the cost to move from current node to neighbour node
    int distanceCost = ManhattenDistance(
        nodePositionArray[ graphIndexArray[ currentNodeIndex ] ] , graphNeighbourPos );

    int tentativeCost = gCostArray[ currentNodeIndex ] + distanceCost;

    if ( tentativeCost < gCostArray[ localNeighbourIndex ] )
    {
        int newHCost = ManhattenDistance( graphNeighbourPos , endPosition );

        parentArray[ localNeighbourIndex ] = currentNodeIndex;
        hCostArray[ localNeighbourIndex ] = newHCost;
        gCostArray[ localNeighbourIndex ] = tentativeCost;
        fCostArray[ localNeighbourIndex ] = tentativeCost + hCost;

        if ( !openArray[ localNeighbourIndex ] )
        {
            openArray[ localNeighbourIndex ] = true;
            openSet.Enqueue( localNeighbourIndex , fCostArray[ localNeighbourIndex ] );
        }
    }
}*/

// Pathfinding Job

/*int2 neighbourKey =
    graphNodeNeighbourLists[ graphIndexArray[ currentNodeIndex ] ];
for ( int i = neighbourKey.x; i < neighbourKey.y; i++ )
{
    int graphNeighbour = graphNodeNeighbours[ i ];
    int2 graphNeighbourPos = graphNodePositions[ graphNeighbour ];

    // Get the local neighbour index
    int neighbourCol = graphNeighbourPos.x - clusterPosition.x * clusterSize;
    int neighbourRow = graphNeighbourPos.y - clusterPosition.y * clusterSize;
    int localNeighbourIndex = neighbourCol + neighbourRow * clusterSize;

    // Skip if its closed (already searched)
    if ( closedArray[ localNeighbourIndex ] )
        continue;

    // Calculate the cost to move from current node to neighbour node
    int distanceCost = ManhattenDistance(
        graphNodePositions[ graphIndexArray[ currentNodeIndex ] ] , graphNeighbourPos );

    int tentativeCost = gCostArray[currentNodeIndex] + distanceCost;

    if ( tentativeCost < gCostArray[ localNeighbourIndex ] )
    {
        int newHCost = ManhattenDistance( graphNeighbourPos , endPosition );

        parentArray[ localNeighbourIndex ] = currentNodeIndex;
        hCostArray[ localNeighbourIndex ] = newHCost;
        gCostArray[ localNeighbourIndex ] = tentativeCost;
        fCostArray[ localNeighbourIndex ] = tentativeCost + hCost;

        if ( !openArray[ localNeighbourIndex ] )
        {
            openArray[ localNeighbourIndex ] = true;
            openSet.Enqueue( localNeighbourIndex , fCostArray[ localNeighbourIndex ] );

            /*if ( localNeighbourIndex >= 0 && localNeighbourIndex < pathNodeArraySize && fCostArray[ localNeighbourIndex ] > 0 )
            {
                openArray[ localNeighbourIndex ] = true;
                openSet.Enqueue( localNeighbourIndex , fCostArray[ localNeighbourIndex ] );
            }
            else
            {
                UnityEngine.Debug.Log( "Failed to enqueue low level neighbour" );
                openSet.Dispose();
                localIndexArray.Dispose();
                graphIndexArray.Dispose();
                parentArray.Dispose();
                hCostArray.Dispose();
                gCostArray.Dispose();
                fCostArray.Dispose();
                openArray.Dispose();
                closedArray.Dispose();

                return new NativeList<float2>( Allocator.Temp );
            }
        }
    }
}*/
