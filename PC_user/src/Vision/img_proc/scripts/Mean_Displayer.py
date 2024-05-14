#!/usr/bin/env python

import cv2
import numpy as np
import yaml
import sys

from yaml.loader import SafeLoader

#### Recibe como argumentos el color y la media que se busca
#### $	python Mean_Displayer.py Red Mean_L1
#### Solo hay que modificar ya sea el color o el numero de media:
#### Red/Green/etc       Mean_L1/Mean_L2/etc

def main():
	piece = 0
	with open('means_SolidColor_Pieces_mod_mod.yaml', 'r') as f:
		data = list(yaml.load_all(f, Loader=SafeLoader))

	if len(sys.argv) > 1:
		for i in data:
			if sys.argv[1] == i['PieceColor']:
				piece = i['PieceCode']
				print"Looking for:", i['PieceColor'], "Piece"
				print "\n"
	else: 
		print "No recibi argumento, buscare la pieza roja c:"

	if len(sys.argv) > 2:
		desired_mean = sys.argv[2]
	else:
		desired_mean = 'Mean_L1'
		print "No recibi argumento, buscare la Mean_L1"


	PieceInfo = data[piece]
	print(PieceInfo)
	print("\n")

	mean_bgr = 255*np.ones((400,400,3),np.uint8)
	mean_hsv = 255*np.ones((400,400,3),np.uint8)
	mean_ln = tuple([float(i) for i in PieceInfo[desired_mean].split(',')])

	print(mean_ln)


	mean_hsv[:,:,0]=mean_ln[0]*np.ones((400,400),np.uint8)
	mean_hsv[:,:,1]=mean_ln[1]*np.ones((400,400),np.uint8)
	mean_hsv[:,:,2]=mean_ln[2]*np.ones((400,400),np.uint8)
	mean_bgr = cv2.cvtColor(mean_hsv, cv2.COLOR_HSV2BGR)

	img_bgr_name = sys.argv[1]+'_'+sys.argv[2]+'_toprint.png'


	while True:
		cv2.imwrite(img_bgr_name,mean_bgr)
		cv2.imshow("Mean Obtained", mean_bgr)
		if cv2.waitKey(100) & 0xFF == 27:
			break
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()

