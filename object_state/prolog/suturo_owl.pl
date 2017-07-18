:- module(suturo_owl,
	  [ owl_instance_from_class/2,
	  	get_class_facts/3
	  ]).

:- rdf_meta owl_instance_from_class(r,?).

owl_instance_from_class(Class, Individual) :-
	rdf_instance_from_class(Class,Individual),
	foreach(get_class_facts(Class,Property,Value),
		rdf_assert(Individual,Property,Value)).

get_class_facts(Class, Property, Value) :- 
		rdf_has(Class,rdfs:subClassOf,A),
		rdf_has(A,owl:hasValue,Value),
		rdf_has(A,owl:onProperty,Property).

