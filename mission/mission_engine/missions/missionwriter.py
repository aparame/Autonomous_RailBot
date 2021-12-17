import simplejson
import constants
f = open('./mission0.txt','w')
simplejson.dump([[constants.MOVE,10], [constants.MOVE,-10]],f)
f.close()
