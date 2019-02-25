#!/usr/bin/env python3
# -*-coding:utf-8 -*

from time import sleep
from copy import deepcopy
import sys
from threading import Thread, RLock
try:
	from inputs import get_gamepad
except ImportError as e:
	print("""You must install the "inputs" library, using "python3 -m pip install inputs" before using this programm""")
	print("""If pip is not installed for python3, use "sudo apt-get install python3-pip", then "python3 -m pip install --upgrade pip", and, finally, use the abose command""")
	sys.exit(0)


def map(x0, xMin0, xMax0, xMin1 = 0, xMax1 = 100):
	"""This function return a value which"""
	return xMin1 + (((x0 -xMin0) * (xMax1 - xMin1)) / (xMax0 - xMin0))


class DictionnaireOrdonne:
	"""Notre dictionnaire ordonné. L'ordre des données est maintenu et il peut donc, contrairement aux dictionnaires usuels, être trié ou voir l'ordre de ses données inversées"""
	def __init__(self, base={}, **donnees):
		"""Constructeur de notre objet. Il peut ne prendre aucun paramètre (dans ce cas, le dictionnaire sera vide) ou construire un dictionnaire remplis grâce :
			-   au dictionnaire 'base' passé en premier paramètre ;
			-   aux valeurs que l'on retrouve dans 'donnees'."""
		self._cles = [] # Liste contenant nos clés
		self._valeurs = [] # Liste contenant les valeurs correspondant à nos clés

		# On vérifie que 'base' est un dictionnaire exploitable
		if type(base) not in (dict, DictionnaireOrdonne):
			raise TypeError("le type attendu est un dictionnaire (usuel ou ordonne)")
		# On récupère les données de 'base'
		for cle in base.keys():
			self[cle] = base[cle]
		# On récupère les données de 'donnees'
		for cle in donnees.keys():
			self[cle] = donnees[cle]

	def __repr__(self):
		"""Représentation de notre objet. C'est cette chaîne qui sera affichée quand on saisit directement le dictionnaire dans l'interpréteur, ou en utilisant la fonction 'repr'"""
		string = "{"
		for key, value in self._virtualController.items():
			string += str(key) + " : " + str(value) + ", "
		string = string[:-2]
		string += "}"
		return string
	def __str__(self):
		"""Fonction appelée quand on souhaite afficher le dictionnaire grâce à la fonction 'print' ou le convertir en chaîne grâce au constructeur 'str'. On redirige sur __repr__"""
		return repr(self)

	def __len__(self):
		"""Renvoie la taille du dictionnaire"""
		return len(self._cles)
	def __contains__(self, cle):
		"""Renvoie True si la clé est dans la liste des clés, False sinon"""
		return cle in self._cles

	def __getitem__(self, cle):
		"""Renvoie la valeur correspondant à la clé si elle existe, lève une exception KeyError sinon"""
		if cle not in self._cles:
			raise KeyError("La clé {0} ne se trouve pas dans le dictionnaire".format(cle))
		else:
			indice = self._cles.index(cle)
		return self._valeurs[indice]
	def __setitem__(self, cle, valeur):
		"""Méthode spéciale appelée quand on cherche à modifier une cle présente dans le dictionnaire. Si la clé n'est pas présente, on l'ajoute à la fin du dictionnaire"""
		if cle in self._cles:
			indice = self._cles.index(cle)
			self._valeurs[indice] = valeur
		else:
			self._cles.append(cle)
			self._valeurs.append(valeur)
	def __delitem__(self, cle):
		"""Méthode appelée quand on souhaite supprimer une clé"""
		if cle not in self._cles:
			raise KeyError("La clé {0} ne se trouve pas dans le dictionnaire".format(cle))
		else:
			indice = self._cles.index(cle)
			del self._cles[indice]
			del self._valeurs[indice]

	def __iter__(self):
		"""Méthode de parcours de l'objet. On renvoie l'itérateur des clés"""
		return iter(self._cles)
	def items(self):
		"""Renvoie un générateur contenant les couples (cle, valeur)"""
		for i, cle in enumerate(self._cles):
			valeur = self._valeurs[i]
			yield (cle, valeur)
	def keys(self):
		"""Cette méthode renvoie la liste des clés"""
		return list(self._cles)
	def values(self):
		"""Cette méthode renvoie la liste des valeurs"""
		return list(self._valeurs)

class XboxOneController(Thread):
	"""This class represent a Xbox One controller"""
	_lock = RLock()
	_lastMaximas = {
		"JL": {
			"U": -32768,
			"D": 32767,
			"L": -32768,
			"R": 32767
		},
		"JR": {
			"U": -32768,
			"D": 32767,
			"L": -32768,
			"R": 32767
		},
		"L2": {
			"R": 0,
			"P": 1023
		},
		"R2": {
			"R": 0,
			"P": 1023
		}
	}
	_simpleButtons = {
		"BTN_SOUTH": "A",
		"BTN_EAST": "B",
		"BTN_NORTH": "X",
		"BTN_WEST": "Y",

		"BTN_TL": "L1",
		"BTN_TR": "R1",

		"BTN_THUMBL": "L3",
		"BTN_THUMBR": "R3",

		"BTN_START": "Start",
		"BTN_SELECT": "Select",
		"BTN_MODE": "Central"
	}
	_directionalButtons = {
		"ABS_HAT0X": ("L", "R"),
		"ABS_HAT0Y": ("U", "D")
	}
	_analogValues = {
		"ABS_X": ("JL", 0),
		"ABS_Y": ("JL", 1),
		"ABS_RX": ("JR", 0),
		"ABS_RY": ("JR", 1),
		"ABS_Z": ("L2", 2),
		"ABS_RZ": ("R2", 2)
	}

	def __init__(self, JLU = 100, JLD = -100, JLL = -100, JLR = 100, JRU = 100, JRD = -100, JRL = -100, JRR = 100, LR = 0, LP = 100, RR = 0, RP = 100):
		"""Constructeur de la classe, chaque valeur analogiques peuvent être personnalisées ou mises en % par défaut :
			JLU -> valeur donnée lorsque le joystick gauche est vers le haut
			JLD -> valeur donnée lorsque le joystick gauche est vers le bas
			JLL -> valeur donnée lorsque le joystick gauche est vers la gauche
			JLR -> valeur donnée lorsque le joystick gauche est vers la droite

			JRU -> valeur donnée lorsque le joystick droit est vers le haut
			JRD -> valeur donnée lorsque le joystick droit est vers le bas
			JRL -> valeur donnée lorsque le joystick droit est vers la gauche
			JRR -> valeur donnée lorsque le joystick droit est vers la droite

			LR -> valeur donnée lorsque la gachette gauche (L2) est relâchée
			LP -> valeur donnée lorsque la gachette gauche (L2) est enfoncée

			RR -> valeur donnée lorsque la gachette droite (R2) est relâchée
			RP -> valeur donnée lorsque la gachette droite (R2) est enfoncée
		"""
		Thread.__init__(self)
		self._running = False
		self._newMaximas = {
			"JL": {
				"U": JLU,
				"D": JLD,
				"L": JLL,
				"R": JLR
			},
			"JR": {
				"U": JRU,
				"D": JRD,
				"L": JRL,
				"R": JRR
			},
			"L2": {
				"R": LR,
				"P": LP
			},
			"R2": {
				"R": RR,
				"P": RP
			}
		}
		self._virtualController = DictionnaireOrdonne()
		self._virtualController["A"] = False
		self._virtualController["B"] = False
		self._virtualController["X"] = False
		self._virtualController["Y"] = False

		self._virtualController["U"] = False
		self._virtualController["D"] = False
		self._virtualController["L"] = False
		self._virtualController["R"] = False

		self._virtualController["L1"] = False
		self._virtualController["L2"] = self._newMaximas["L2"]["R"]
		self._virtualController["L3"] = False
		self._virtualController["R1"] = False
		self._virtualController["R2"] = self._newMaximas["R2"]["R"]
		self._virtualController["R3"] = False

		self._virtualController["JL"] = [(self._newMaximas["JL"]["L"] + self._newMaximas["JL"]["R"])/2, (self._newMaximas["JL"]["U"] + self._newMaximas["JL"]["D"])/2]
		self._virtualController["JR"] = [(self._newMaximas["JR"]["L"] + self._newMaximas["JR"]["R"])/2, (self._newMaximas["JR"]["U"] + self._newMaximas["JR"]["D"])/2]

		self._virtualController["Start"] = False
		self._virtualController["Select"] = False
		self._virtualController["Central"] = False

	def __repr__(self):
		"""Méthode qui modifie la façon dont l'objet est affiché quand on tape directement son nom"""
		string = ""
		for key, value in self._virtualController.items():
			string += str(key) + " : " + str(value) + "\n"
		return string

	def __str__(self):
		"""Méthode appelée pour la conversion en str de l'objet"""
		return repr(self)

	def __getattr__(self, nom_attr):
		"""Méthode appelée lorsque le programme tente d'accéder à un attribut qui n'existe pas."""
		print("Attention ! L'attribut", nom_attr, "n'existe pas pour les objets de la classe XboxOneRemote")
	def __delattr__(self, nom_attr):
		"""Méthode permettant d'empêcher la suppression des attributs de la classe."""
		raise TypeError("'XboxOneRemote' object does not support item deletion")

	def __getitem__(self, index):
		"""Méthode permettant de récupérer les états de la mannette en utilisant l'objet comme étant la variable _virutalController."""
		return deepcopy(self._virtualController[index])
	def __setitem__(self, index, valeur):
		"""Méthode permettant d'empêcher un programme externe de modifier les états de la mannette."""
		raise TypeError("'XboxOneRemote' object does not support state assignment")
	def __delitem__(self, index):
		"""Méthode permettant d'empêcher un programme externe de supprimer les états de la mannette."""
		raise TypeError("'XboxOneRemote' object does not support state deletion")

	def __contains__(self, val):
		"""Méthode permettant d'utiliser le mot-clé in sur un objet de la classe, renvoie vers la même méthode sur le dictionnaire _virtualController."""
		return self._virtualController.__contains__(val)
	def __len__(self):
		"""Méthode permettant d'utiliser le mot-clé len sur un objet de la classe, renvoie vers la même méthode sur le dictionnaire _virtualController."""
		return self._virtualController.__len__()

	def _setattr(self, nom_attr, val_attr):
		"""Méthode permettant d'empêcher la modification des attributs de la classe."""
		raise TypeError("'XboxOneRemote' object does not support item assignment")
	def _getNewMaximas(self):
		return deepcopy(self._newMaximas)
	newMaximas = property(_getNewMaximas, _setattr, __delattr__)
	def _getVirtualController(self):
		return deepcopy(self._virtualController)
	virtualController = property(_getVirtualController, _setattr, __delattr__)
	def _getLastMaximas(self):
		return deepcopy(self._lastMaximas)
	lastMaximas = property(_getLastMaximas, _setattr, __delattr__)
	def _getSimpleButtons(self):
		return deepcopy(self._simpleButtons)
	simpleButtons = property(_getSimpleButtons, _setattr, __delattr__)
	def _getDirectionalButtons(self):
		return deepcopy(self._directionalButtons)
	directionalButtons = property(_getDirectionalButtons, _setattr, __delattr__)
	def _getAnalogValues(self):
		return deepcopy(self._analogValues)
	analogValues = property(_getAnalogValues, _setattr, __delattr__)

	def _listenController(self):
		events = get_gamepad()
		with self._lock:
			for event in events:
				for key, value in self._simpleButtons.items():
					if(event.code == key):
						self._virtualController[value] = bool(event.state)
				for key, value in self._directionalButtons.items():
					if(event.code == key):
						for i in range(len(value)):
							self._virtualController[value[i]] = (False, False, True)[i + event.state]
				for key, value in self._analogValues.items():
					if(event.code == key):
						if(value[1] != 2):
							self._virtualController[value[0]][value[1]] = map(event.state, self._lastMaximas[value[0]][("L", "D")[value[1]]], self._lastMaximas[value[0]][("R", "U")[value[1]]], self._newMaximas[value[0]][("L", "D")[value[1]]], self._newMaximas[value[0]][("R", "U")[value[1]]])
						else:
							self._virtualController[value[0]] = map(event.state, self._lastMaximas[value[0]]["R"], self._lastMaximas[value[0]]["P"], self._newMaximas[value[0]]["R"], self._newMaximas[value[0]]["P"])
	def run(self):
		self._running = True
		while self._running:
			self._listenController()
	def stop(self):
		self._running = False
		self.join()



if __name__ == '__main__':
	pass
	#while(1):
		#events = get_gamepad()
		#for event in events:
			#print(event.code, event.state, event.ev_type)
