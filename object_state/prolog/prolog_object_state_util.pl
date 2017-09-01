/** <module> prolog_object_state_util

@author Lukas Samel, Michael Speer, Sascha Jongebloed 
@license BSD
*/

% defining functions
:- module(prolog_object_state_util,
    [
      get_class_name/2,
      multiple_objects_name/2,
      get_max_num/2,
      get_type_num/2,
      create_object_name/2,
      same_dimensions/2,
      same_position/3,
      is_different_from_by/3
    ]).

:- dynamic
        isConnected/2.

:- rdf_meta get_type_num(r,?),
            create_object_name(r,?).

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


%parse libraries
:- owl_parse('package://knowrob_common/owl/knowrob.owl').
:- owl_parse('package://knowrob_map_data/owl/ccrl2_semantic_map.owl').
:- owl_parse('package://object_state/owl/test_actions.owl').


%% get_class_name(+Type, -ClassName)
% MSp
% converts first letter of Type into capital letter
get_class_name(Type, ClassName) :-
    not((var(Type), var(ClassName))),
    sub_atom(Type,0,1,_,C), char_code(C,I), 96<I, I<123
    -> J is I-32, char_code(D,J), sub_atom(Type,1,_,0,Sub), atom_concat(D,Sub,ClassName)
    ; ClassName = Type.


%% multiple_objects_name(+Type, -NameNum)
% MSp
% creates object name depending on how many other objects of same type exist
% @param Name type leading to NameNum by appending iterating number
% @param NameNum is the unique name of the new object in KB, e.g. cake32
multiple_objects_name(Type, NameNum) :-
    (get_max_num(Type, Num), number(Num)
    -> Number is Num+1; Number is 1),
    atom_concat(Type, Number, NameNum).


%% get_max_num(+Type, - Number)
% MSp
% gets maximum number from name of object type
get_max_num(Type, Number) :-
    setof(Num,get_type_num(Type,Num),NumberList),
    max_list(NumberList,Number).


%% get_type_num(+Type, -Number)
% MSp
% helper funciton for to get max Number in NameNum
get_type_num(Type, Number) :-
    holds(Obj,knowrob:'nameOfObject',FullName),
    strip_literal_type(FullName,FullNameStripped),
    create_object_name(NameNum,FullNameStripped),
    atom_concat(Type, NumChar, NameNum),
    atom_number(NumChar, Number).


%% strip_name_num(+NameNum, -Name)
% MSp
% removes appended numbers from object name
% @param NameNum is the name with appended type count, e.g. cake32
% @param Name is the type of the object
strip_name_num(NameNum, Name) :-
    sub_atom(NameNum, _,1,0,L), atom_number(L, _)
    -> sub_string(NameNum,0,_,1, Sub), strip_name_num(Sub, Name);
      Name = NameNum.


%% create_object_name(+Name,-FullName) is det.
% LSa
% Appends Name to the knowrob domain.
% @param Name content to be appended to the namespace
% @preturns FullName the concatenated string
create_object_name(Name, FullName) :-
    atom_concat('http://knowrob.org/kb/knowrob.owl#', Name, FullName).


is_different_from_by(OldValue,NewValue,Tolerance) :-
    (number(OldValue),number(NewValue)->
      true;
      false,debug('is_different_from_by without number')),
    OldValueLow is OldValue - Tolerance,
    OldValueHigh is OldValue + Tolerance,
    (NewValue < OldValueLow; NewValue > OldValueHigh).


%% same_dimensions(+[PrevDim], +[CurDim])
%MSp
%@param PrevDim dimensions of object at previous timestamp
%@param CurDim  dimensions of object at current  timestamp
same_dimensions([E|PrevDim], CurDim) :-
    length(PrevDim, 0) -> member(E, CurDim);
    same_dimensions([E], CurDim), same_dimensions(PrevDim, CurDim).


%% same_position(+[PrevPos], +[CurPos], +[Dimensions])
%MSp
%@param PrevPos position of object at previous timestamp
%@param CurPos  position of object at current  timestamp
%@param Dimensions tolerance allowed between positions to consider unchanged is maximum dimension of object
same_position(PrevPos, CurPos, Dimensions) :-
    euclidean_squared_dist(PrevPos, CurPos, Dist),
    max_member(Dmax, Dimensions),
    DmaxSquared is Dmax ^ 2.0,
    Dist < DmaxSquared.


%% euclidean_dist(+[PointA], +[PointB], -Dist)
% MSp
euclidean_squared_dist(PointA, PointB, Dist) :-
    length(PointA, Len), length(PointB, Len) ->
    sqr_sum(PointA, PointB, SqrSum), Dist is SqrSum; false.


%% sqr_sum(+[A1|An], +[B1|Bn], -SqrSum)
% MSp
sqr_sum([A1|An], [B1|Bn], SqrSum) :-
    length(An, 0) ->
    SqrSum is ((A1-B1)^2.0);
    sqr_sum(An, Bn, Sum),
    SqrSum is Sum + ((A1-B1)^2.0).