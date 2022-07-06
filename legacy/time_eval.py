
import time

start = time.time()
count = 0
for i in range(500):
    for j in range(500):
        for k in range(600):
            count += 1

end = time.time()
print(end - start)
