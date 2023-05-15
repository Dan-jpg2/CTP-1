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
frontCone = short[90:-90] # short[int(samples/(8/3)):int(-samples/(8/3))]
print("\nfrontCone:", frontCone, len(frontCone))

rightCone = short[150:] # int(samples/(8/5))
print("\nrightCone:", rightCone, len(rightCone))

leftCone = short[:90] # int(samples/(8/3))
print("\nleftCone:", leftCone, len(leftCone))

print("\nleftPart right: ", rightCone[:int(len(rightCone)/2)])
print("\nrightPart right: ", rightCone[int(len(rightCone)/2):])

print("\nleftPart left: ", leftCone[:int(len(rightCone)/2)])
print("\nrightPart left: ", leftCone[int(len(rightCone)/2):])

print(240/3)