/** <module> prolog_object_state

@author Sascha Jongebloed 
@license BSD
*/

% defining functions
:- module(catering_queries,
    [
      get_current_edibles/1,
      get_current_edibles_help/1
    ]).

:- rdf_meta get_current_edibles.

get_current_edibles(FoodClass) :-
      setof(A,get_current_edibles_help(FoodClass),_).

get_current_edibles_help(FoodClass) :-
      current_temporal_part(Obj,_),
      owl_has(Obj,rdf:type,FoodClass),
      owl_subclass_of(FoodClass, knowrob:'FoodOrDrink').

