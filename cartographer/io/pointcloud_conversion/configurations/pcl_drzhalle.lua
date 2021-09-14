

options = {
    generateCubicPointcloud = false,

    pointcloudPath = "/Downloads/Halle-DRZ-Modell-innen-Flug1-2020-12-09.ply",

    uniformDownSample = true,
    sampleRateUniformDownSample = 100,

    voxelDownSample = true,
    voxelSizeVoxelDownSample = 0.1,

    removeRadiusOutliers = true,
    sphereSizeRadiusOutliers = 0.3,
    neighborsInSphereRadiusOutlier = 2,

    cutRoofZAxis = false,
    cutoffSize = 3.0,

    normalOrientationNearestNeighbours = 40,

    absoluteHighResVoxelSize = 0.1,
    absoluteLowResVoxelSize = 0.3,
    absoluteTruncationDistance = 0.3,

    imageSliceIndex = -55,






}

return options