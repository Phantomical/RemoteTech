using System;
using CommNet.Network;
using RemoteTech.SimpleTypes;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using UnityEngine;

namespace RemoteTech.Jobs;

internal interface IRangeModel
{
    double MaxDistance(double r1, double r2);
}

[Flags]
internal enum OmniFlags : byte
{
    /// <summary>
    ///
    /// </summary>
    BaseStation = 0x1,
    ControlPoint = 0x2,

    MissionControl = 0x3,
}

/// <summary>
/// An omnidirectional antenna on a vessel.
/// </summary>
internal struct JobAntenna
{
    /// <summary>
    /// A guid identifying the actual antenna object that this job antenna
    /// corresponds to.
    /// </summary>
    public Guid guid;

    /// <summary>
    /// The vessel that this antenna is attached to.
    /// </summary>
    public int vessel;

    /// <summary>
    /// Extra information about this omni antenna.
    /// </summary>
    public OmniFlags flags;

    /// <summary>
    /// The range of this antenna.
    /// </summary>
    public double omni;

    /// <summary>
    /// The target for this antenna:
    /// *  0 => no target
    /// * <0 => target is celestial body at index (-target - 1)
    /// * >0 => target is vessel at index (target - 1)
    /// </summary>
    public int target;

    /// <summary>
    /// The range of this antenna.
    /// </summary>
    public double dish;

    public double cosAngle;
}

/// <summary>
/// A base station omnidirectional antenna. These are always connected and are
/// used as the roots when computing whether vessels are connected to mission
/// control.
/// </summary>
internal struct JobBaseStationOmniAntenna
{
    public Vector3d position;
    public double range;
}

internal struct JobSattelite
{
    public Guid guid;
    public Vector3d position;
    public IntRange antennas;
    public bool command;
}

internal struct JobCelestialBody
{
    public Vector3d position;
    public double radius;
}

internal static class NetworkUpdateCommon
{
    /// <summary>Checks the maximum range achievable by two satellites.</summary>
    /// <returns>The maximum range, including sanity limits.</returns>
    /// <param name="rangeFunc">A function that takes two antenna ranges and returns a joint range.</param>
    /// <param name="range1">The range of the first satellite.</param>
    /// <param name="range2">The range of the second satellite.</param>
    /// <param name="clamp1">The maximum factor by which the first range can be boosted.</param>
    /// <param name="clamp2">The maximum factor by which the second range can be boosted.</param>
    internal static double CheckRange<T>(
        T model,
        double range1,
        double clamp1,
        double range2,
        double clamp2
    )
        where T : IRangeModel
    {
        var clamp = Math.Min(range1 * clamp1, range2 * clamp2);
        return Math.Min(model.MaxDistance(range1, range2), clamp);
    }

    internal static bool HasLineOfSight(
        Vector3d posA,
        Vector3d posB,
        NativeArray<JobCelestialBody> bodies
    )
    {
        const double MIN_HEIGHT = 5.0;

        foreach (var body in bodies)
        {
            Vector3d bodyFromA = body.position - posA;
            Vector3d bFromA = posB - posA;

            // Is body at least roughly between satA and satB?
            if (Vector3d.Dot(bodyFromA, bFromA) <= 0)
                continue;
            Vector3d bFromANorm = bFromA.normalized;
            if (Vector3d.Dot(bodyFromA, bFromANorm) >= bFromA.magnitude)
                continue;

            // Above conditions guarantee that Vector3d.Dot(bodyFromA, bFromANorm) * bFromANorm
            // lies between the origin and bFromA
            Vector3d lateralOffset = bodyFromA - Vector3d.Dot(bodyFromA, bFromANorm) * bFromANorm;
            if (lateralOffset.magnitude < body.radius - MIN_HEIGHT)
                return false;
        }

        return true;
    }
}

/// <summary>
/// Update the positions of each dish attached to a vessel and compute the
/// maximum dish range for each vessel.
/// </summary>
[BursstCompile]
internal struct UpdateAntennaDataJob : IJobParallelFor
{
    [ReadOnly]
    public NativeArray<JobSattelite> vessels;

    [ReadOnly]
    public NativeArray<JobCelestialBody> bodies;

    [NoAlias]
    public NativeArray<JobAntenna> antennas;

    [WriteOnly]
    public NativeArray<Vector3d> directions;

    public void Execute(int index)
    {
        var vessel = vessels[index];

        foreach (var antennaIdx in vessel.antennas)
        {
            var antenna = antennas[antennaIdx];
            antenna.vessel = index;

            Vector3d target;
            if (antenna.target == 0)
                target = vessel.position;
            else if (antenna.target > 0) // vessel
                target = vessels[antenna.target - 1].position;
            else // celestial body
                target = bodies[-antenna.target - 1].position;

            var rel = target - vessel.position;
            var mag = rel.magnitude;
            if (mag == 0.0)
                rel = Vector3d.zero;
            else
                rel *= 1.0 / mag;

            antennas[antennaIdx] = antenna;
            directions[antennaIdx] = rel;
        }
    }
}

/// <summary>
/// Compute the pairwise connectivity between each vessel.
/// </summary>
internal struct ComputeVesselConnectivity
{
    [ReadOnly]
    public NativeArray<JobAntenna> antennas;

    [ReadOnly]
    public NativeArray<Vector3d> directions;

    [ReadOnly]
    public NativeArray<JobCelestialBody> bodies;

    [ReadOnly]
    public NativeArray<JobSattelite> vessels;

    /// <summary>
    /// What is the effective distance between the dishes. If they are not
    /// connected then this is set to <c>double.MaxValue</c>.
    /// </summary>
    [WriteOnly]
    public NativeArray<double> distances;

    [WriteOnly]
    public NativeArray<LinkType> types;

    /// <summary>
    /// Indicates whether this antenna is connected to any other antennas.
    /// </summary>
    [WriteOnly]
    public NativeArray<bool> connected;

    public double omniClamp;
    public double dishClamp;

    public double multipleAntennaMultiplier;

    public void Execute<T>(T model, int index)
        where T : IRangeModel
    {
        int x = index % vessels.Length;
        int y = index / vessels.Length;

        if (x > y)
            return;
        if (x == y)
        {
            types[index] = LinkType.None;
            distances[index] = 0.0;
            return;
        }

        var vesselA = vessels[x];
        var vesselB = vessels[y];
        var link = ComputeLinkInfo(model, vesselA, vesselB);

        types[index] = link.type;
        distances[index] = link.length;
    }

    struct LinkInfo
    {
        public LinkType type;
        public double length;

        public static LinkInfo NotConnected =>
            new() { type = LinkType.None, length = double.MaxValue };
    }

    LinkInfo ComputeLinkInfo<T>(T model, JobSattelite vesselA, JobSattelite vesselB)
        where T : IRangeModel
    {
        if (!NetworkUpdateCommon.HasLineOfSight(vesselA.position, vesselB.position, bodies))
            return LinkInfo.NotConnected;

        var maxOmniA = GetMaxOmniRange(vesselA.antennas);
        var maxOmniB = GetMaxOmniRange(vesselB.antennas);
        var maxDishA = GetMaxConnectedDishRange(
            vesselA.antennas,
            vesselA.position,
            vesselB.position
        );
        var maxDishB = GetMaxConnectedDishRange(
            vesselB.antennas,
            vesselB.position,
            vesselA.position
        );
        var bonusA = maxOmniA += GetMultipleAntennaBonus(vesselA.antennas, maxOmniA);
        var bonusB = maxOmniB += GetMultipleAntennaBonus(vesselB.antennas, maxOmniB);

        maxOmniA += bonusA;
        maxOmniB += bonusB;

        var distance = Vector3d.Distance(vesselA.position, vesselB.position);
        // Can at least one of each category of antenna on the vessel reach the other one?
        // csharpier-ignore-start
        bool omnisA  = CheckRange(model, maxOmniA, omniClamp, maxOmniB, omniClamp) >= distance
                    || CheckRange(model, maxOmniA, omniClamp, maxDishB, dishClamp) >= distance;
        bool dishesA = CheckRange(model, maxDishA, dishClamp, maxOmniB, omniClamp) >= distance
                    || CheckRange(model, maxDishA, dishClamp, maxDishB, dishClamp) >= distance;
        bool dishesB = CheckRange(model, maxDishB, dishClamp, maxOmniA, omniClamp) >= distance
                    || CheckRange(model, maxDishB, dishClamp, maxDishA, dishClamp) >= distance;
        // csharpier-ignore-end

        if (!omnisA && !dishesA)
            return LinkInfo.NotConnected;

        var type = dishesA && dishesB ? LinkType.Dish : LinkType.Omni;

        // Figure out which antennas are
        foreach (var antennaIdx in vesselA.antennas)
        {
            var an = antennas[antennaIdx];
            // csharpier-ignore-start
            bool conn = CheckRange(model, an.omni + bonusA, omniClamp, maxOmniB, omniClamp) >= distance
                     || CheckRange(model, an.omni + bonusA, omniClamp, maxDishB, dishClamp) >= distance
                     || CheckRange(model, an.dish,          dishClamp, maxOmniA, omniClamp) >= distance
                     || CheckRange(model, an.dish,          dishClamp, maxDishB, dishClamp) >= distance;
            // csharpier-ignore-end

            // This way this only ever gets set to true so race conditions are
            // not an issue (on x86, at least).
            if (conn)
                connected[antennaIdx] = true;
        }

        return new() { type = type, length = distance };
    }

    double GetMultipleAntennaBonus(IntRange range, double maxOmni)
    {
        if (multipleAntennaMultiplier <= 0.0)
            return 0.0;

        double total = 0.0;
        foreach (var index in range)
            total += antennas[index].omni;

        return (total - maxOmni) * multipleAntennaMultiplier;
    }

    double GetMaxConnectedDishRange(IntRange range, Vector3d source, Vector3d target)
    {
        var max = 0.0;
        var reldir = (target - source).normalized;

        foreach (var index in range)
        {
            var antenna = antennas[index];
            if (antenna.dish <= 0.0)
                continue;
            if (antenna.target == 0)
                continue;

            if (Vector3d.Dot(reldir, directions[index]) > antenna.cosAngle)
                continue;

            max = Math.Max(antenna.dish, max);
        }

        return max;
    }

    double GetMaxOmniRange(IntRange range)
    {
        var max = 0.0;
        foreach (var index in range)
            max = Math.Max(antennas[index].omni, max);
        return max;
    }

    double CheckRange<T>(T model, double range1, double clamp1, double range2, double clamp2)
        where T : IRangeModel =>
        NetworkUpdateCommon.CheckRange(model, range1, clamp1, range2, clamp2);
}

internal struct ComputeBaseStationConnectivity
{
    [ReadOnly]
    public NativeArray<JobAntenna> antennas;

    [ReadOnly]
    public NativeArray<Vector3d> directions;

    [ReadOnly]
    public NativeArray<JobCelestialBody> bodies;
}

internal struct ClearArrayJob<T> : IJob
    where T : unmanaged
{
    [WriteOnly]
    public NativeArray<T> array;

    public unsafe void Execute()
    {
        UnsafeUtility.MemClear(array.GetUnsafePtr(), sizeof(T) * array.Length);
    }
}

internal struct FillArrayJob<T> : IJob
    where T : unmanaged
{
    [WriteOnly]
    public NativeArray<T> array;

    public T value;

    public void Execute()
    {
        for (int i = 0; i < array.Length; ++i)
            array[i] = value;
    }
}

