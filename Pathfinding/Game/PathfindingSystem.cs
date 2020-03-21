using System.Diagnostics;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

[AlwaysUpdateSystem]
public class PathfindingSystem : ComponentSystem
{
    private const int HEURISTIC_BIAS = 1;
    private const int MAX_JOBS_PER_FRAME = 8;
    private const int JOB_BATCH_SIZE = 2;

    private PathfindingGraph graph;

    private NativeList<Entity> jobEntities = new NativeList<Entity>( Allocator.Persistent );
    private NativeList<float2> jobStartPositions = new NativeList<float2>( Allocator.Persistent );
    private NativeList<float2> jobEndPositions = new NativeList<float2>( Allocator.Persistent );

    private Queue<Entity> entityQueue = new Queue<Entity>();
    private Queue<float2> startPositionQueue = new Queue<float2>();
    private Queue<float2> endPositionQueue = new Queue<float2>();

    private Stopwatch sw;
    private bool showMarkers = true;

    protected override void OnStartRunning()
    {
        base.OnStartRunning();
        sw = new Stopwatch();
        graph = GameHandler.instance.pathfindingGraph;
    }
    protected override void OnUpdate()
    {
        sw.Reset();
        sw.Start();

        bool showTime = false;

        jobEntities.Clear();
        jobStartPositions.Clear();
        jobEndPositions.Clear();

        Entities.ForEach( ( Entity entity , ref PathFinding_Orders pathFindingOrders , ref Translation translation ) =>
        {
            entityQueue.Enqueue( entity );
            startPositionQueue.Enqueue( new float2( translation.Value.x , translation.Value.z ) );
            endPositionQueue.Enqueue( pathFindingOrders.targetPosition );

            PostUpdateCommands.RemoveComponent<PathFinding_Orders>( entity );
        } );

        int numEntitiesWaiting = entityQueue.Count;
        if ( numEntitiesWaiting > 0 )
        {
            int numJobs = MAX_JOBS_PER_FRAME;
            if ( numEntitiesWaiting < 4 )
                numJobs = numEntitiesWaiting;

            showTime = true;

            jobEntities.Clear();
            jobStartPositions.Clear();
            jobEndPositions.Clear();

            for ( int i = 0; i < numJobs; i++ )
            {
                jobEntities.Add( entityQueue.Dequeue() );
                jobStartPositions.Add( startPositionQueue.Dequeue() );
                jobEndPositions.Add( endPositionQueue.Dequeue() );
            }

            FindPathJob job = new FindPathJob
            {
                // GRID DATA
                neighbourOffsetArray = graph.neighbourOffsetArray ,
                heuristicBias = HEURISTIC_BIAS ,
                graphCellLength = graph.numCellsAcross ,
                graphCellSize = graph.cellSize ,
                graphClusterLength = graph.clusterSize ,
                graphNumClusters = graph.numClustersAcross ,
                // CLUSTER
                graphClusterEdgesLists = graph.clusterEdges ,
                graphIntraEdges = graph.intraEdges ,
                // EDGE
                graphEdgeNeighbourList = graph.edgeNeighbourKeys ,
                graphEdgePathList = graph.pathKeysPerEdge ,
                graphEdgePaths = graph.edgePaths ,
                graphEdgePathPositions = graph.edgePathPositions ,
                graphEdgeNeighbours = graph.edgeNeighbours ,
                // NODE
                graphInterEdges = graph.interEdges ,
                graphNodeToEdge = graph.nodeToEdge ,
                graphNodePositions = graph.nodePositions ,
                graphNodeWalkables = graph.nodeWalkables ,
                // ENTITY DATA
                entities = jobEntities.AsArray() ,
                startWorldPositions = jobStartPositions.AsArray() ,
                endWorldPositions = jobEndPositions.AsArray() ,
                // WRITABLE ENTITY DATA
                pathPositionBuffer = GetBufferFromEntity<Path_Position>() ,
                pathIndexComponentData = GetComponentDataFromEntity<Path_Index>()
            };

            JobHandle jobHandle = job.Schedule( numJobs , JOB_BATCH_SIZE );
            jobHandle.Complete();

            sw.Stop();
            if ( showTime )
                UnityEngine.Debug.Log( sw.Elapsed );
        }
            /*if ( showMarkers )
            {
                BufferFromEntity<Path_Position> pathPositionBuffer = GetBufferFromEntity<Path_Position>();
                Entities.ForEach( ( Entity entity , ref Path_Index index ) =>
                {
                    if ( pathPositionBuffer[ entity ].Length > 0 )
                    {
                        for ( int i = 0; i < pathPositionBuffer[ entity ].Length; i++ )
                        {
                            Transform t = Object.Instantiate( graph.marker ).transform;
                            float x = pathPositionBuffer[ entity ][ i ].position.x;
                            float z = pathPositionBuffer[ entity ][ i ].position.y;
                            t.position = new float3( x , 2 , z );
                        }
                    }
                } );
            }*/
        //}
    }
    protected override void OnDestroy()
    {
        jobEntities.Dispose();
        jobStartPositions.Dispose();
        jobEndPositions.Dispose();
        base.OnDestroy();
    }

    [BurstCompile] private struct FindPathJob : IJobParallelFor
    {
        // Grid Data
        [ReadOnly] public int heuristicBias;
        [ReadOnly] public int graphCellLength;
        [ReadOnly] public int graphNumClusters;
        [ReadOnly] public int graphCellSize;
        [ReadOnly] public int graphClusterLength;
        // CLUSTER
        [ReadOnly] public NativeArray<int2> graphClusterEdgesLists;
        //[ReadOnly] public NativeArray<Blittable_Bool> graphClustersDense;
        [ReadOnly] public NativeArray<int> graphIntraEdges;
        [ReadOnly] public NativeArray<int> graphInterEdges;
        // EDGE
        [ReadOnly] public NativeArray<int2> graphEdgeNeighbourList; // NodeLength
        [ReadOnly] public NativeArray<int> graphEdgeNeighbours;
        [ReadOnly] public NativeArray<int2> graphEdgePathList; // NodeLength
        [ReadOnly] public NativeArray<int2> graphEdgePaths;
        [ReadOnly] public NativeArray<float2> graphEdgePathPositions;
        // NODE
        [ReadOnly] public NativeArray<int2> neighbourOffsetArray;
        [ReadOnly] public NativeArray<int> graphNodeToEdge; // NodeLength
        [ReadOnly] public NativeArray<int2> graphNodePositions; // NodeLength
        [ReadOnly] public NativeArray<byte> graphNodeWalkables; // NodeLength
        // Entity Data
        [ReadOnly] public NativeArray<float2> startWorldPositions;
        [ReadOnly] public NativeArray<float2> endWorldPositions;
        [ReadOnly] public NativeArray<Entity> entities;
        // Entity Data To Write To
        [NativeDisableContainerSafetyRestriction] public BufferFromEntity<Path_Position> pathPositionBuffer;
        [NativeDisableContainerSafetyRestriction] public ComponentDataFromEntity<Path_Index> pathIndexComponentData;

        public void Execute( int jobIndex )
        {
            // Get the current entity
            Entity entity = entities[ jobIndex ];
            pathPositionBuffer[ entity ].Clear();
            pathIndexComponentData[ entity ] = new Path_Index { index = -1 };

            // Get grid positions from world positions (float2 -> int2)
            float2 startGridPositionAsFloat = new float2( 
                startWorldPositions[ jobIndex ].x / graphCellSize , 
                startWorldPositions[ jobIndex ].y / graphCellSize );
            float2 endGridPositionAsFloat = new float2( 
                endWorldPositions[ jobIndex ].x / graphCellSize , 
                endWorldPositions[ jobIndex ].y / graphCellSize );
            int2 startGridPosition = new int2( 
                ( int ) math.round( startGridPositionAsFloat.x ) , 
                ( int ) math.round( startGridPositionAsFloat.y ) );
            int2 endGridPosition = new int2( 
                ( int ) math.round( endGridPositionAsFloat.x ) , 
                ( int ) math.round( endGridPositionAsFloat.y ) );

            // Get the cluster positions from the grid positions
            int2 startClusterPosition = new int2(
                startGridPosition.x / graphClusterLength ,
                startGridPosition.y / graphClusterLength );
            int2 endClusterPosition = new int2(
                endGridPosition.x / graphClusterLength ,
                endGridPosition.y / graphClusterLength );

            // If the start and end positions are in different clusters
            if ( !startClusterPosition.Equals( endClusterPosition ) )
            {                    
                // Get the start and end node indexes from their grid cluster positions
                int clusterIndex = startClusterPosition.x + startClusterPosition.y * graphNumClusters;
                int2 clusterEdgesKey = graphClusterEdgesLists[ clusterIndex ];

                // Check every edge in the cluster to see if we can reach it, and add it to the list if we can
                NativeList<int> startEdges = new NativeList<int>( Allocator.Temp );
                for ( int edgeIndex = clusterEdgesKey.x; edgeIndex < clusterEdgesKey.y; edgeIndex++ )
                    startEdges.Add( graphIntraEdges[ edgeIndex ] );

                // If there are no paths from start position to any of the edges in current cluster, break out
                if ( startEdges.Length == 0 )
                {
                    pathIndexComponentData[ entity ] = new Path_Index { index = -1 };
                    startEdges.Dispose();
                    return;
                }
                else
                {
                    NativeList<float2> highLevelPath = FindHighLevelPathInGraph( startEdges , startGridPosition , endGridPosition );

                    if ( highLevelPath.Length > 0 )
                    {
                        int2 endOfHLPath = new int2(
                            ( int ) highLevelPath[ 0 ].x / graphCellSize ,
                            ( int ) highLevelPath[ 0 ].y / graphCellSize );
                        int2 beginningOfHLPath = new int2(
                            ( int ) highLevelPath[ highLevelPath.Length - 1 ].x / graphCellSize ,
                            ( int ) highLevelPath[ highLevelPath.Length - 1 ].y / graphCellSize );

                        NativeList<float2> finalPath =
                            FindLowLevelPathInCluster( endOfHLPath , endGridPosition , endClusterPosition );
                        NativeList<float2> pathToHLPath =
                            FindLowLevelPathInCluster( startGridPosition , beginningOfHLPath , startClusterPosition );

                        for ( int i = 0; i < highLevelPath.Length; i++ )
                            finalPath.Add( highLevelPath[ i ] );
                        for ( int i = 0; i < pathToHLPath.Length; i++ )
                            finalPath.Add( pathToHLPath[ i ] );
                        // Write final path to entity buffer
                        for ( int i = 0; i < finalPath.Length; i++ )
                            pathPositionBuffer[ entity ].Add( new Path_Position { position = finalPath[ i ] } );

                        pathIndexComponentData[ entity ] =
                            new Path_Index { index = pathPositionBuffer[ entity ].Length - 1 };

                        finalPath.Dispose();
                        pathToHLPath.Dispose();
                        highLevelPath.Dispose();
                        startEdges.Dispose();
                        return;
                    }
                    else
                    {
                        pathIndexComponentData[ entity ] = new Path_Index { index = -1 };
                        startEdges.Dispose();
                        highLevelPath.Dispose();
                        return;
                    }
                }
            }
            else // OTHERWISE, the start and end positions are in the same cluster, so just do a quick low-level a* search
            {
                NativeList<float2> path = 
                    FindLowLevelPathInCluster( startGridPosition , endGridPosition , startClusterPosition );

                if ( path.Length > 0 )
                {
                    for ( int i = 0; i < path.Length; i++ )
                        pathPositionBuffer[ entity ].Add( new Path_Position { position = path[ i ] } );

                    pathIndexComponentData[ entity ] = 
                        new Path_Index { index = pathPositionBuffer[ entity ].Length - 1 };
                }
                else
                {
                    pathIndexComponentData[ entity ] = 
                        new Path_Index { index = -1 };
                }

                path.Dispose();
                return;
            }
        }

        private NativeList<float2> FindLowLevelPathInCluster( int2 startPosition , int2 endPosition , int2 clusterPosition )
        {
            #region DATA SETUP

            int PATH_NODE_ARRAY_SIZE = graphClusterLength * graphClusterLength;

            // Node sets
            NativeMinHeap openSet = new NativeMinHeap();
            openSet.Initialize( PATH_NODE_ARRAY_SIZE , -1 , PATH_NODE_ARRAY_SIZE + 1 );
            NativeArray<int> localIndexArray = new NativeArray<int>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> graphIndexArray = new NativeArray<int>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> parentArray = new NativeArray<int>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> hCostArray = new NativeArray<int>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> gCostArray = new NativeArray<int>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> fCostArray = new NativeArray<int>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<Blittable_Bool> openArray = new NativeArray<Blittable_Bool>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<Blittable_Bool> closedArray = new NativeArray<Blittable_Bool>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );

            // Initialize nodes from ReadOnly grid array
            for ( int localRow = 0; localRow < graphClusterLength; localRow++ )
            {
                for ( int localCol = 0; localCol < graphClusterLength; localCol++ )
                {
                    int localIndex = localCol + localRow * graphClusterLength;
                    int graphCol = localCol + graphClusterLength * clusterPosition.x;
                    int graphRow = localRow + graphClusterLength * clusterPosition.y;
                    int graphArrayIndex = graphCol + graphRow * graphCellLength;

                    localIndexArray[ localIndex ] = localIndex;
                    graphIndexArray[ localIndex ] = graphArrayIndex;
                    parentArray[ localIndex ] = -1;
                    gCostArray[ localIndex ] = int.MaxValue;
                    openArray[ localIndex ] = false;
                    closedArray[ localIndex ] = false;
                }
            }

            // Get and cache the start and end pathNodeIndices
            int endNodeX = endPosition.x - clusterPosition.x * graphClusterLength;
            int endNodeY = endPosition.y - clusterPosition.y * graphClusterLength;
            int startNodeX = startPosition.x - clusterPosition.x * graphClusterLength;
            int startNodeY = startPosition.y - clusterPosition.y * graphClusterLength;
            int endNodeIndex = endNodeX + endNodeY * graphClusterLength;
            int startNodeIndex = startNodeX + startNodeY * graphClusterLength;

            // Initialize the starting pathNode
            int hCost = ManhattenDistance( graphNodePositions[ graphIndexArray[ startNodeIndex ] ] , endPosition );
            gCostArray[ startNodeIndex ] = 0;
            hCostArray[ startNodeIndex ] = hCost;
            fCostArray[ startNodeIndex ] = hCost;
            openArray[ startNodeIndex ] = true;

            // Add the starting node to the open set
            openSet.Enqueue( startNodeIndex , hCost );

            #endregion
            #region SEARCH GRAPH

            while ( openSet.Length > 0 )
            {
                // Cache the pathNodeIndex we are working with during this iteration
                int currentNodeIndex =
                    openSet.DequeueMin( localIndexArray , closedArray );

                openArray[ currentNodeIndex ] = false;
                closedArray[ currentNodeIndex ] = true;

                // Break if we reached our goal
                if ( currentNodeIndex == endNodeIndex )
                    break;

                int2 currentNodePosition =
                    graphNodePositions[ graphIndexArray[ currentNodeIndex ] ];

                for ( int nIndex = 0; nIndex < neighbourOffsetArray.Length; nIndex++ )
                {
                    int2 neighbourPosition =
                        currentNodePosition + neighbourOffsetArray[ nIndex ];

                    if ( !ValidateGridPosition( neighbourPosition , clusterPosition , graphClusterLength ) )
                        continue;

                    int graphNeighbour =
                        neighbourPosition.x + neighbourPosition.y * graphCellLength;

                    if ( graphNodeWalkables[ graphNeighbour ] > 0 )
                        continue;

                    // Get the local neighbour index
                    int2 localNeighbourPosition = new int2(
                        neighbourPosition.x - clusterPosition.x * graphClusterLength ,
                        neighbourPosition.y - clusterPosition.y * graphClusterLength );
                    int localNeighbourIndex =
                        localNeighbourPosition.x + localNeighbourPosition.y * graphClusterLength;

                    // Skip if its closed (already searched)
                    if ( closedArray[ localNeighbourIndex ] )
                        continue;

                    // Calculate the cost to move from current node to neighbour node
                    int distanceCost =
                        ManhattenDistance( currentNodePosition , neighbourPosition );
                    int tentativeCost =
                        gCostArray[ currentNodeIndex ] + distanceCost;

                    if ( tentativeCost < gCostArray[ localNeighbourIndex ] )
                    {
                        int newHCost = ManhattenDistance( neighbourPosition , endPosition );

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
                }
            }

            #endregion
            #region TRACE PATH

            NativeList<int2> aStarPath = new NativeList<int2>( Allocator.Temp );
            NativeList<float2> waypoints = new NativeList<float2>( Allocator.Temp );
            int nodeIndex = endNodeIndex;

            if ( parentArray[ endNodeIndex ] == -1 )
            {
                openSet.Dispose();
                localIndexArray.Dispose();
                graphIndexArray.Dispose();
                parentArray.Dispose();
                hCostArray.Dispose();
                gCostArray.Dispose();
                fCostArray.Dispose();
                openArray.Dispose();
                closedArray.Dispose();
                aStarPath.Dispose();
                return waypoints;
            }
            else
            {
                waypoints.Add( new float2(
                    endPosition.x * graphCellSize ,
                    endPosition.y * graphCellSize ) );
                aStarPath.Add( endPosition );

                while ( parentArray[ nodeIndex ] != -1 )
                {
                    aStarPath.Add( graphNodePositions[ graphIndexArray[ parentArray[ nodeIndex ] ] ] );
                    /*waypoints.Add( new float2(
                        graphNodePositions[ graphIndexArray[ parentArray[ nodeIndex ] ] ].x * graphCellSize ,
                        graphNodePositions[ graphIndexArray[ parentArray[ nodeIndex ] ] ].y * graphCellSize ) );*/
                    nodeIndex = parentArray[ nodeIndex ];
                }
            }

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
                        int graphNodeArrayIndex = aStarPath[ stopIndex ].x + aStarPath[ stopIndex ].y * graphCellLength;

                        int2 start = aStarPath[ fromIndex ];
                        int2 end = aStarPath[ currentIndex ];

                        if ( !LOSBetween2Nodes( start , end ) )
                        {
                            float2 worldPosition = new float2(
                                graphNodePositions[ graphNodeArrayIndex ].x * graphCellSize ,
                                graphNodePositions[ graphNodeArrayIndex ].y * graphCellSize );

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

            openSet.Dispose();
            localIndexArray.Dispose();
            graphIndexArray.Dispose();
            parentArray.Dispose();
            hCostArray.Dispose();
            gCostArray.Dispose();
            fCostArray.Dispose();
            openArray.Dispose();
            closedArray.Dispose();
            aStarPath.Dispose();
            return waypoints;

            #endregion
        }
        private NativeList<float2> FindHighLevelPathInGraph( NativeList<int> startNodes , int2 startPosition , int2 endPosition )
        {
            #region DATA SETUP

            int EDGE_ARRAY_SIZE = graphIntraEdges.Length;

            // Node sets
            NativeMinHeap openSet = new NativeMinHeap(); openSet.Initialize( EDGE_ARRAY_SIZE , -1 , EDGE_ARRAY_SIZE + 10 );
            NativeArray<int> graphNodeIntraIndexArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> graphNodeInterIndexArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> parentEdgeArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> parentPathIndexArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> hCostArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> gCostArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> fCostArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<Blittable_Bool> openArray = new NativeArray<Blittable_Bool>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<Blittable_Bool> closedArray = new NativeArray<Blittable_Bool>( EDGE_ARRAY_SIZE , Allocator.Temp );

            void DisposeNativeContainers()
            {
                openSet.Dispose();
                graphNodeIntraIndexArray.Dispose();
                graphNodeInterIndexArray.Dispose();
                parentEdgeArray.Dispose();
                parentPathIndexArray.Dispose();
                hCostArray.Dispose();
                gCostArray.Dispose();
                fCostArray.Dispose();
                openArray.Dispose();
                closedArray.Dispose();
            }

            // Initialize default array values
            for ( int i = 0; i < EDGE_ARRAY_SIZE; i++ )
            {
                graphNodeIntraIndexArray[ i ] = graphIntraEdges[ i ];
                graphNodeInterIndexArray[ i ] = graphInterEdges[ i ];
                parentEdgeArray[ i ] = -1;
                parentPathIndexArray[ i ] = -1;
                gCostArray[ i ] = int.MaxValue;
                openArray[ i ] = false;
                closedArray[ i ] = false;
            }

            // Initialize the starting nodes(edges) from the parameter
            for ( int i = 0; i < startNodes.Length; i++ )
            {
                int nodeIndex = startNodes[ i ];
                int edgeIndex = graphNodeToEdge[ nodeIndex ];

                hCostArray[ edgeIndex ] = ManhattenDistance( graphNodePositions[ nodeIndex ] , endPosition );
                gCostArray[ edgeIndex ] = ManhattenDistance( startPosition , graphNodePositions[ nodeIndex ] );
                fCostArray[ edgeIndex ] = gCostArray[ edgeIndex ] + hCostArray[ edgeIndex ];

                openSet.Enqueue( edgeIndex , fCostArray[ edgeIndex ] );
            }
            
            // So we can test when we have reached the end
            int2 endClusterPosition = new int2(
                endPosition.x / graphClusterLength ,
                endPosition.y / graphClusterLength );

            int endEdgeIndex = -1;

            #endregion
            #region FIND PATH

            while ( openSet.Length > 0 )
            {
                int currentEdge = openSet.DequeueMin( closedArray );
                int currentInterNode = graphNodeInterIndexArray[ currentEdge ];

                int2 clusterPosition = new int2(
                    graphNodePositions[ currentInterNode ].x / graphClusterLength ,
                    graphNodePositions[ currentInterNode ].y / graphClusterLength );

                if ( clusterPosition.Equals( endClusterPosition ) )
                {
                    endEdgeIndex = currentEdge;
                    break;
                }

                openArray[ currentEdge ] = false;
                closedArray[ currentEdge ] = true;

                int2 interNodeNeighbours = 
                    graphEdgeNeighbourList[ currentInterNode ];
                for ( int i = interNodeNeighbours.x; i < interNodeNeighbours.y; i++ )
                {
                    int neighbourNode = graphEdgeNeighbours[ i ];
                    int neighbourEdge = graphNodeToEdge[ neighbourNode ];

                    if ( closedArray[ neighbourEdge ] )
                        continue;

                    int distanceCost = ManhattenDistance( 
                        graphNodePositions[ currentInterNode ] , graphNodePositions[ neighbourNode ] );
                    int tentativeCost = 
                        distanceCost + gCostArray[ currentEdge ];

                    if ( tentativeCost < gCostArray[ neighbourEdge ] )
                    {
                        int newHCost = ManhattenDistance( graphNodePositions[ neighbourNode ] , endPosition );
                        parentEdgeArray[ neighbourEdge ] = currentEdge;
                        parentPathIndexArray[ neighbourEdge ] = i;
                        hCostArray[ neighbourEdge ] = newHCost;
                        gCostArray[ neighbourEdge ] = tentativeCost;
                        fCostArray[ neighbourEdge ] = newHCost + tentativeCost;

                        if ( !openArray[ neighbourEdge ] )
                        {
                            openArray[ neighbourEdge ] = true;
                            openSet.Enqueue( neighbourEdge , fCostArray[ neighbourEdge ] );
                        }
                    }
                }
            }

            #endregion
            #region TRACE PATH

            NativeList<float2> pathPositions = new NativeList<float2>( Allocator.Temp );

            if ( endEdgeIndex == -1 )
            {
                DisposeNativeContainers();
                return pathPositions;
            }
            else
            {
                // Add the interNode position of the last edge to the path
                int interNodeIndex = 
                    graphNodeInterIndexArray[ endEdgeIndex ];
                float2 endOfPath = new float2(
                    graphNodePositions[ interNodeIndex ].x * graphCellSize ,
                    graphNodePositions[ interNodeIndex ].y * graphCellSize );
                pathPositions.Add( endOfPath );

                // Trace the path
                int edgeIndex = endEdgeIndex;
                while ( parentEdgeArray[ edgeIndex ] != -1 )
                {
                    int2 pathPositionKey = 
                        graphEdgePaths[ parentPathIndexArray[ edgeIndex ] ];

                    for ( int i = pathPositionKey.x; i < pathPositionKey.y; i++ )
                        pathPositions.Add( graphEdgePathPositions[ i ] );

                    edgeIndex = parentEdgeArray[ edgeIndex ];
                }

                // Add the internode of the first edge to the path
                int interEdgeIndex = graphNodeToEdge[ graphNodeInterIndexArray[ edgeIndex ] ];
                int lastIndex = graphNodeInterIndexArray[ interEdgeIndex ];
                float2 startPos = new float2(
                    graphNodePositions[ lastIndex ].x * graphCellSize ,
                    graphNodePositions[ lastIndex ].y * graphCellSize );
                pathPositions.Add( startPos );

                // Return
                DisposeNativeContainers();
                return pathPositions;
            }

            #endregion
        }
        private bool LOSBetween2Nodes( int2 start , int2 end )
        {
            int2 dif = new int2(
                math.abs( end.x - start.x ) ,
                math.abs( end.y - start.y ) );
            int2 pos = new int2(
                start.x ,
                start.y );
            int2 inc = new int2(
                ( end.x > start.x ) ? 1 : -1 ,
                ( end.y > start.y ) ? 1 : -1 );

            int numIterations = 1 + dif.x + dif.y;
            int error = dif.x - dif.y;
            int walkable = 0;
            bool prevY = false;
            int numSameDir = 0;

            if ( error <= 0 )
                prevY = true;

            dif.x *= 2;
            dif.y *= 2;

            for ( ; numIterations > 0; --numIterations )
            {
                walkable += graphNodeWalkables[ pos.x + pos.y * graphCellLength ];

                if ( error > 0 ) // more steps in x
                {
                    pos.x += inc.x;
                    error -= dif.y;
                }
                else // more steps in y
                {
                    pos.y += inc.y;
                    error += dif.x;
                }

                /*if ( numSameDir < 1 )
                {
                    if ( prevY )
                        walkable += graphNodeWalkables[ ( pos.x - inc.x ) + pos.y * graphCellLength ];
                    else
                        walkable += graphNodeWalkables[ pos.x + ( pos.y - inc.y ) * graphCellLength ];

                    if ( error > 0 ) // more steps in x
                    {
                        pos.x += inc.x;

                        if ( !prevY )
                            numSameDir++;
                        else
                            numSameDir = 0;

                        prevY = false;
                        error -= dif.y;
                    }
                    else // more steps in y
                    {
                        pos.y += inc.y;

                        if ( prevY )
                            numSameDir++;
                        else
                            numSameDir = 0;

                        prevY = true;
                        error += dif.x;
                    }
                }*/
            }

            return walkable == 0;
        }
        private int ManhattenDistance( int2 positionA , int2 positionB )
        {
            int2 distance = positionB - positionA;
            return math.abs( distance.x ) + math.abs( distance.y );
        }
        private bool ValidateGridPosition( int2 position , int2 clusterPosition , int2 clusterSize )
        {
            int2 min = new int2(
                clusterPosition.x * graphClusterLength ,
                clusterPosition.y * graphClusterLength );

            return
                position.x >= min.x && position.x < min.x + clusterSize.x &&
                position.y >= min.y && position.y < min.y + clusterSize.y;
        }
    }
}