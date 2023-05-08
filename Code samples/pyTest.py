a = []
for i in range(360):
    a.append(i+1)

samples = len(a)
print("Samples:", samples)
print("int(samples/3):", int(samples/3))
short = a[2*int(samples/3):]
short.extend(a[:int(samples/3)])
#for i in range(int(samples/3)):
#    short.append(a[i])
print(short)
print("Len of that:", len(short))
samples = len(short)
frontCone = short[int(samples/3):int(-samples/3)]
print("\nfrontCone:", frontCone)

rightCone = short[int(2*samples/3):]
print("\nrightCone:", rightCone)

leftCone = short[:int(samples/3)]
print("\nleftCone:", leftCone)

print("\nleftPart right: ", rightCone[:int(len(frontCone)/2)])
print("\nrightPart right: ", rightCone[int(len(frontCone)/2):])

print("\nleftPart left: ", leftCone[:int(len(frontCone)/2)])
print("\nrightPart left: ", leftCone[int(len(frontCone)/2):])