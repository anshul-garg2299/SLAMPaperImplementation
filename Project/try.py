import csv
i = [3,4,5,7]
with open('data.csv','w',newline = '') as f:
	csv_writer = csv.writer(f)
	for k in range(4):
		csv_writer.writerow(i)
	