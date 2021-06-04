

options = {
    generateCubicPointcloud = false,

    pointcloudPath = "/Downloads/Halle-DRZ-Modell-innen-Flug1-2020-12-09.ply",

    uniformDownSample = true,
    sampleRateUniformDownSample = 100,

    voxelDownSample = true,
    voxelSizeVoxelDownSample = 0.1,

    removeRadiusOutliers = true,
    sphereSizeRadiusOutliers = 0.3,
    neighborsInSphereRadiusOutlier = 5,

    cutRoofZAxis = false,
    cutoffSize = 3.0,

    normalOrientationNearestNeighbours = 40,

    absoluteVoxelSize = 0.1,
    absoluteTruncationDistance = 0.5,
    maxTSDFWeight = 10.0,

    imageSliceIndex = -55,






}

return options