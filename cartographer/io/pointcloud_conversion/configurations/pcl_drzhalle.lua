

options = {
    generateCubicPointcloud = false,

    pointcloudPath = "/Downloads/Halle-DRZ-Modell-innen-Flug1-2020-12-09.ply",

    uniformDownSample = true,
    sampleRateUniformDownSample = 100,

    voxelDownSample = true,
    voxelSizeVoxelDownSample = 0.1,

    removeRadiusOutliers = true,
    sphereSizeRadiusOutliers = 1.0,
    neighborsInSphereRadiusOutlier = 3,

    cutRoofZAxis = true,
    cutoffSize = 3.0,

    normalOrientationNearestNeighbours = 40,

    absoluteVoxelSize = 0.5,
    absoluteTruncationDistance = 1.0,
    maxTSDFWeight = 10.0,






}

return options