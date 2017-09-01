/** <module> prolog_object_state_close

@author Lukas Samel, Michael Speer, Sascha Jongebloed 
@license BSD
*/

% defining functions
:- module(prolog_object_state_close,
    [
      close_object/1,
      close_corresponding_physical_parts/1,
      close_object_help/1
    ]).

:- dynamic
        isConnected/2.

:- rdf_meta close_object(r),
      close_corresponding_physical_parts(r),
      close_object_help(r).

%importing external libraries
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('knowrob_coordinates')).
:- use_module(library('knowrob_temporal')).
:- use_module(library('knowrob_objects')).
:- use_module(library('knowrob_owl')).
:- use_module(library('srdl2')).
:- use_module(library('rdfs_computable')).
:- use_module(library('swrl')).


%registering namespace
%#:- rdf_db:rdf_register_ns(knowrob,  'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
%#:- rdf_db:rdf_register_ns(srdl2, 'http://knowrob.org/kb/srdl2.owl#', [keep(true)]).
%#:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).
%#:- rdf_db:rdf_register_ns(srdl2cap, 'http://knowrob.org/kb/srdl2-cap.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(suturo_obj, 'package://object_state/owl/suturo_object.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(suturo_act, 'package://object_state/owl/suturo_actions.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(suturo_cap, 'http://knowrob.org/kb/suturo-cap.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pepper, 'http://knowrob.org/kb/pepper.owl', [keep(true)]).
:- rdf_db:rdf_register_ns(test_actions, 'http://knowrob.org/kb/test_actions.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(test_actions, 'http://knowrob.org/kb/test_actions.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(test_swrl, 'http://knowrob.org/kb/swrl_test#', [keep(true)]).
:- rdf_db:rdf_register_prefix(swrl, 'http://www.w3.org/2003/11/swrl#', [keep(true)]).

%parse libraries
:- owl_parse('package://knowrob_common/owl/knowrob.owl').
:- owl_parse('package://knowrob_map_data/owl/ccrl2_semantic_map.owl').
:- owl_parse('package://knowrob_common/owl/swrl_test.owl').
:- owl_parse('package://object_state/owl/test_actions.owl').


close_object(TypeName) :-
  forall(
    holds(ObjInst,knowrob:'typeOfObject',TypeName),
    close_object_help(ObjInst)
  ).


close_object(ObjectName) :-
  (atom_concat('http://knowrob.org/kb/knowrob.owl#', Name, ObjectName) -> 
    NewObjectName = ObjectName;
    atom_concat('http://knowrob.org/kb/knowrob.owl#', ObjectName, NewObjectName)),
  forall(
    holds(ObjInst,knowrob:'nameOfObject',NewObjectName),
    close_object_help(ObjInst)
  ).

close_object_help(ObjInst) :-
  debug(ObjInst),
  ignore((close_corresponding_physical_parts(ObjInst))),
  current_time(Now),
  forall(holds(ObjInst,P,O),
  ignore(assert_temporal_part_end(ObjInst,P,O,Now))).

close_corresponding_physical_parts(ObjInst) :-
  holds(ObjInst,knowrob:'physicalParts',_),
  forall(
    holds(ObjInst,knowrob:'physicalParts',PhysicalPart),
    ignore(close_object_help(PhysicalPart))
  ).