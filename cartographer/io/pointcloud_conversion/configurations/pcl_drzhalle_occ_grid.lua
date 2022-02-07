

options = {
    generateTSDF = false,
    outputName = "DRZHalleOccupancyGrid",

    generateCubicPointcloud = false,

    pointcloudPath = "/Downloads/Halle-DRZ-Modell-innen-Flug1-2020-12-09.ply",

    uniformDownSample = false,
    sampleRateUniformDownSample = 100,

    voxelDownSample = true,
    voxelSizeVoxelDownSample = 0.1,

    removeRadiusOutliers = true,
    sphereSizeRadiusOutliers = 0.3,
    neighborsInSphereRadiusOutlier = 2,

    cutRoofZAxis = false,
    cutoffSize = 3.0,

    normalOrientationNearestNeighbours = 100,

    absoluteHighResVoxelSize = 0.1,
    absoluteLowResVoxelSize = 0.4,
    relativeHighResTruncationDistance = 2.5,
    relativeLowResTruncationDistance = 2.5,

    saveSlicesAsPNG = false,
    imageSliceIndex = -55,






}

return options