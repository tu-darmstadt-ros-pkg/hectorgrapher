

options = {
    generateTSDF = true,        -- otherwise an occupancy grid is generated
    outputName = "DRZHalleTSDF",

    --pointcloudPath = "/Downloads/Halle-DRZ-Modell-innen-Flug1-2020-12-09-part-part.ply",
    pointcloudPath = "/Downloads/Halle-DRZ-Modell-innen-Flug1-2020-12-09.ply",

    scale = false,

    uniformDownSample = true,
    sampleRateUniformDownSample = 10,

    voxelDownSample = true,
    voxelSizeVoxelDownSample = 0.08,

    removeRadiusOutliers = true,
    sphereSizeRadiusOutliers = 0.2,
    neighborsInSphereRadiusOutlier = 19,

    cutRoofZAxis = false,
    cutoffSize = 3.0,

    normalOrientationNearestNeighbours = 30,

    absoluteHighResVoxelSize = 0.1,
    absoluteLowResVoxelSize = 0.4,
    relativeHighResTruncationDistance = 2.5,
    relativeLowResTruncationDistance = 2.5,

    saveSlicesAsPNG = false,
    imageSliceIndex = -55,

    numberOfSubmaps = 10,






}

return options