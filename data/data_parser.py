import csv

data = []
with open('./data.txt', 'r') as f:
    csv_reader = csv.reader(f, delimiter=',', quotechar=']')
    count = 0
    for line in csv_reader:
        count += 1
        if count%2 != 0:
            line[0] = line[0][1:]
            line[-1] = line[-1][:-1]
            data.append(line)
    
with open('./data.csv', 'wb') as f:
    writer = csv.writer(f, delimiter=',')
    for i in data:
        writer.writerow(i)