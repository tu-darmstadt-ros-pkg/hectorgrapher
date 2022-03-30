

options = {
    generateTSDF = true,
    outputName = "TestWorldTSDF",

    pointcloudPath = "/Downloads/test_world.ply",

    scale = false,
    scaleRate = 2,

    uniformDownSample = true,
    sampleRateUniformDownSample = 2,

    voxelDownSample = false,
    voxelSizeVoxelDownSample = 0.1,

    removeRadiusOutliers = false,
    sphereSizeRadiusOutliers = 0.3,
    neighborsInSphereRadiusOutlier = 2,

    cutRoofZAxis = false,
    cutoffSize = 3.0,

    normalOrientationNearestNeighbours = 10,

    absoluteHighResVoxelSize = 0.1,
    absoluteLowResVoxelSize = 0.4,
    relativeHighResTruncationDistance = 2.5,
    relativeLowResTruncationDistance = 2.5,

    saveSlicesAsPNG = false,
    imageSliceIndex = -55,

    numberOfSubmaps = 1,

}

return options