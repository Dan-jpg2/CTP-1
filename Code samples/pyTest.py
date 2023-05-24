

a = []
for i in range(360):
    a.append(i+1)

samples = len(a)
print("Samples:", samples)
print("int(samples/4):", int(samples/4))

print("a[:int(samples/4)]", a[:int(samples/4)])
print("a[int(samples/4):2*int(samples/4)]", a[int(samples/4):2*int(samples/4)])
print("a[2*int(samples/4):3*int(samples/4)]", a[2*int(samples/4):3*int(samples/4)])
print("a[3*int(samples/4):]", a[3*int(samples/4):])

new = []
new = a[:int(samples/4)][::-1]
new.extend(a[3*int(samples/4):][::-1])

print(new)