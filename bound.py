MIN = 0
MAX = 1000

inp = [
	[ 200, 500, 500, 800 ],
	[ 0, 200, 300, 1000 ],
	[ 200, 400, 500, 1200 ],
	[ -200, 0, 100, 800 ],
	[ -200, 500, 500, 2000 ],
	[ -5000, 500, 500, 5000 ],
	[ -5000, 500, 500, 1000 ],
	[ 0, 500, 500, 5000 ],
]

for i in range(len(inp)):
	min = inp[i][0]
	max = inp[i][0]

	for j in range(1, len(inp[i])):
		if inp[i][j] < min:
			min = inp[i][j]
		if inp[i][j] > max:
			max = inp[i][j]

	r = (max - min)
	if r > (MAX - MIN):
		m = (MAX - MIN) / float(max - min)
	else:
		m = 1.0

	s = 0
	if max*m > MAX:
		s = (MAX - max*m)
	elif min*m < MIN:
		s = (MIN - min*m)

	outp = [int(v*m + s) for v in inp[i]]

	print min, max, r, '%.2f' % m, s, inp[i], outp

