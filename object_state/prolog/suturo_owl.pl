:- module(suturo_owl,
	  [ owl_individual_from_class/2,
	  	get_class_facts/3
	  ]).


owl_individual_from_class(Individual, Class) :-
	owl_individual_of(Individual,Class),
	forall(get_class_facts(Class,Property,Value),
		rdf_assert(Individual,Property,Value)).

get_class_facts(Class, Property, Value) :- 
	rdf_has(Class, owl:subClassOf, _A),
	rdf_has(_A,owl:onProperty,Property), 
	rdf_has(_A, owl:hasValue, Value).
