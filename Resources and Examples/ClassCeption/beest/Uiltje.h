#include "Arduino.h"
#include "Veertje.h"

class Uiltje{

	int honger;
	int kleur;
	Veertje Hoofdveer;
	Veertje Wangveer;

	public:
	Uiltje(int honger_, int kleur_): Hoofdveer(kleur), Wangveer(kleur){

		honger = honger_;
		kleur = kleur_;
	}

	void Vlieg(){


		digitalWrite(5, honger);
		Hoofdveer.Shine();
		Wangveer.Shine();

	}
};