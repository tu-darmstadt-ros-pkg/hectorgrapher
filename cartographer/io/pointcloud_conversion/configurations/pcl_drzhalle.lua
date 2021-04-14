

options = {
    generateCubicPointcloud = false,

    pointcloudPath = "/home/leo/Downloads/Halle-DRZ-Modell-innen-Flug1-2020-12-09.ply",

    uniformDownSample = true,
    sampleRateUniformDownSample = 100,

    removeRadiusOutliers = true,
    sphereSizeRadiusOutliers = 1.0,
    neighborsInSphereRadiusOutlier = 3,

    cutRoofZAxis = true,
    cutoffSize = 5.0,

    normalOrientationNearestNeighbours = 40,

    absoluteVoxelSize = 1.0,
    absoluteTruncationDistance = 1.0,
    maxTSDFWeight = 10.0,






}

return options