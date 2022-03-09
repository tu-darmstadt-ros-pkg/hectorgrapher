

options = {
    generateTSDF = false,
    outputName = "CorridorOccupancyGrid",

    generateCubicPointcloud = false,

    pointcloudPath = "/Downloads/corridor.ply",

    uniformDownSample = false,
    sampleRateUniformDownSample = 100,

    voxelDownSample = false,
    voxelSizeVoxelDownSample = 0.1,

    removeRadiusOutliers = false,
    sphereSizeRadiusOutliers = 0.3,
    neighborsInSphereRadiusOutlier = 2,

    cutRoofZAxis = false,
    cutoffSize = 3.0,

    absoluteHighResVoxelSize = 0.1,
    absoluteLowResVoxelSize = 0.4,
    relativeHighResTruncationDistance = 2.5,
    relativeLowResTruncationDistance = 2.5,

    saveSlicesAsPNG = false,
    imageSliceIndex = -55,







}

return options