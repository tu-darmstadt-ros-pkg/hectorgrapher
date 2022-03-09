

options = {
    generateTSDF = true,        -- otherwise an occupancy grid is generated
    outputName = "myTSDF",

    generateCubicPointcloud = true,
    sidelengthCubicPointcloud = 0.1,
    distancePointsCubicPointcloud = 0.004,
    noiseCubicPointcloud = 0.0,

    pointcloudPath = "../../../../Downloads/Halle-DRZ-Modell-innen-Flug1-2020-12-09.ply",

    uniformDownSample = false,
    sampleRateUniformDownSample = 100,

    voxelDownSample = false,
    voxelSizeVoxelDownSample = 0.1,

    removeRadiusOutliers = false,
    sphereSizeRadiusOutliers = 1.0,
    neighborsInSphereRadiusOutlier = 3,

    cutRoofZAxis = false,
    cutoffSize = 5.0,

    normalOrientationNearestNeighbours = 40,

    absoluteVoxelSize = 1.0,
    absoluteTruncationDistance = 1.0,
    maxTSDFWeight = 10.0,

    saveSlicesAsPNG = false,

    numberOfSubmaps = 1,







}

return options