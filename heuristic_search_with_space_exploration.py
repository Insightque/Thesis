class Explore:
	def Generate_circle(x,y):
	"""adott kezdőpontokkal kör létrehozása
	sugár  = legközelebbi akadály - kocsi mérete"""

	def Overlap(c1,c2):
	"""megnézi hogy 2 kör átfedése megfelelő határok közt van-e?
	 (pl.:kissebb kör sugarának 50%)"""

	def PopLargest(S):
	"""Kiveszi S halmazból a legnagyobb átmérővel rendelkező kört"""

	def PopNearest(S):
	"""Kiveszi S halmazból azt a kört, amely a célhoz leközelebb van"""

	def NotExist(c):
	"""Megnézi hogy c nagymértékbe átfedésbe van-e a bejárt halmazban lévőkkel (S_closed).
	Ha átfedésben van (NotExist == True) akkor azt mellőzzük a redundancia csökkentése miatt"""

	def Expand(c):
	"""Mintavételezi c-t és a mintapontokra meghívja a Generate_circle(x,y)-t"""

	def Space_Explaration(x_start,y_start, x_goal, y_goal ):
		c_start = Generate_circle(xs,ys)
		c_goal = Generate_circle(xe,ye)
		S_closed = 0
		S_open = c_start

		while S_open != 0:
			if Overlap(Nearest(S_open),c_goal):
				return True
			else:
				c_nearest = PopNearest(S_open)
				c_largest = PopLargest(S_open)
				
				if NotExist(c_nearest):
					Sopen = Expand(c_nearest)
					S_closed += c_nearest
				f NotExist(c_largest):
					Sopen = Expand(c_largest)
					S_closed += c_largest
		else:
			return False
			
			
class Search:


	def Expand(q,s):
	"""
	új configurácót generál q-ból s lépésközzel
	A megfelelő s lépésköz megválasztásában a kör mérete segít
	s = min(A*rc,B*dc,smin)
	"""

	def Heuristic_Search(q_start, q_goal, c_i):
		S_closed = 0
		S_open = q_start 		 # kezdő konfiguráció
		i = 0
		
		# amíg van új, bejárandó körök
		while S_open != 0:
			if Top(S_open) == goal:
				return True
			else:
				q = PopTop(S_open)
				if NotExist(q):
					s = UpdateStep(ci,q_goal,q)
					S_open = Expand(q,s) 
					S_closed = q
		return False