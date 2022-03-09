

options = {
    generateTSDF = true,
    outputName = "drzWorldTSDF",

    generateCubicPointcloud = false,

    pointcloudPath = "/Downloads/drz_world.ply",

    uniformDownSample = false,
    sampleRateUniformDownSample = 4,

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

}

return options