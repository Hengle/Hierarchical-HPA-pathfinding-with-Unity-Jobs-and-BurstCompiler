using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Burst;
using Unity.Collections;
using Unity.Transforms;

public class PathFollowJobSystem : JobComponentSystem
{
    // CAN RUN ON DIFFERENT FRAMES FOR DIFFERENT UNITS

    [BurstCompile] private struct PathFollowJob : IJobForEachWithEntity_EBCC<Path_Position , Path_Index , Translation>
    {
        [ReadOnly] public float dt;

        public void Execute( Entity entity , int index , 
            DynamicBuffer<Path_Position> pathPositionBuffer , ref Path_Index pathIndex , ref Translation translation )
        {
            if ( pathIndex.index >= 0 )
            {
                float3 pathPosition = new float3( pathPositionBuffer[ pathIndex.index ].position.x , 2 , pathPositionBuffer[ pathIndex.index ].position.y );
                float distance = math.distance( pathPosition , translation.Value );
                float3 direction = math.normalize( pathPosition - translation.Value );

                if ( math.abs( direction.x ) <= 1 )
                {
                    translation.Value += new float3( direction.x * 15f * dt , 0 , direction.z * 15f * dt );
                } 

                if ( distance <= 0.125f )
                {
                    int newPathIndex = pathIndex.index - 1;
                    pathIndex = new Path_Index { index = newPathIndex };
                }
            }
        }
    }
    
    protected override JobHandle OnUpdate( JobHandle inputDeps )
    {
        PathFollowJob job = new PathFollowJob
        {
            dt = Time.DeltaTime
        };
        JobHandle jobHandle = job.Schedule( this , inputDeps );

        return jobHandle;
    }
}