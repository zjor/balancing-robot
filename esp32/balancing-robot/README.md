# Balancing Robot ESP32 WROOM

## Patching `SparkFun MPU-9250 Digital Motion Processing _DMP_ Arduino Library`

1. Go to `src/util/inv_mpu.h`
2. Add the following lines:
```C
#define min(X,Y) (((X) < (Y)) ? (X) : (Y))
#define max(X,Y) (((X) > (Y)) ? (X) : (Y))
```

## Notes

- `roll` is used for balancing