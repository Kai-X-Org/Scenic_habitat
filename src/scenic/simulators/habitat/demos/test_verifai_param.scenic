param v0 = VerifaiRange(0, 8)
param v1 = VerifaiRange(0, 8)
param v2 = VerifaiRange(0, 8)
param v3 = VerifaiRange(0, 8)
param v4 = VerifaiRange(0, 8)
param v5 = VerifaiRange(0, 8)
param v6 = VerifaiRange(0, 8)
param v7 = VerifaiRange(0, 8)
param v8 = VerifaiRange(0, 8)
param v9 = VerifaiRange(0, 8)
param v10 = VerifaiRange(0, 8)

lst = [globalParameters.v0, globalParameters.v1, globalParameters.v2,
       globalParameters.v3, globalParameters.v4, globalParameters.v5,
       globalParameters.v6, globalParameters.v7, globalParameters.v8, 
       globalParameters.v9, globalParameters.v10]


for v in lst:
    obj = new Object at (v, 0, 0), with shape SpheroidShape(dimensions=(0.5, 0.5, 0.5))


