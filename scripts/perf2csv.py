import re
import sys
import csv
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--output', '-o', default='perf.csv', help='Path to output csv file.')
args = parser.parse_args()

regex_perf = re.compile(r"(.+) performance:\n  - num cycles: (\d+)\n  - MACs: (\d+)\n  - MAC/cycle: (.+)",
                        re.MULTILINE)

data = sys.stdin.read()

csv_list = [["Name", "Latency", "Ops", "Perf"]]

for match in regex_perf.finditer(data):
    csv_list.append([match.group(1), match.group(2), match.group(3), match.group(4)])
    

with open(args.output, "w") as f:
    writer = csv.writer(f)
    writer.writerows(csv_list)
