

options = {
    generateTSDF = true,
    outputName = "bunnyTSDF",

    pointcloudPath = "/Downloads/bunny/reconstruction/bun_zipper.ply",

    scale = true,
    scaleRate = 100,

    uniformDownSample = false,

    voxelDownSample = false,

    removeRadiusOutliers = false,

    cutRoofZAxis = false,

    normalOrientationNearestNeighbours = 50, --9 / 50

    absoluteHighResVoxelSize = 0.2,
    absoluteLowResVoxelSize = 0.8,
    relativeHighResTruncationDistance = 5.0,
    relativeLowResTruncationDistance = 5.0,

    saveSlicesAsPNG = false,
    imageSliceIndex = -10,

    numberOfSubmaps = 1,


}

return options