#include "Arduino.h"

class Veertje{

	int kleur;

	public:
	Veertje(int kleur_){

		kleur = kleur_;
	}

	int Shine(){

		return analogRead(5);
	}
};