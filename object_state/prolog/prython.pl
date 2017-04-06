/*

  Copyright (C) 2017 Sascha Jongebloed
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@author Sascha Jongebloed
@license BSD

*/

:- module(prython,
    [
      py_call_base/5,
      py_call/5,
      py_call/4,
      add_py_path/1,
      remove_py_path/1,
      create_parameter_string/2,
      return_true_type/2,
      string_list_to_list/2,
      string_list_to_list_one/2,
      string_list_to_list_one/6,
      read_lines/2,
      max_depth/2,
      remove_char/3
    ]).

%% py_call_base(+PathTo:string, +ScriptName:string, +FunctionName:string, +Parameter:list, ?Return) is semidet.
%
% Base predicate to call a function of a python file.
%
% @param PathTo The path to the python file
% @param ScriptName The name of the python script
% @param FunctionName The function to be called
% @param Parameter The Parameter for the python function
% @param Return The return value of the python function as a string
%
py_call_base(PathTo,ScriptName,FunctionName,Parameter,Return):-
	working_directory(OldPath,PathTo),
  create_parameter_string(Parameter,ParameterString),
	atomic_list_concat(['import sys;import io; import ',ScriptName,
		';save_out = sys.stdout;sys.stdout = io.BytesIO(); ret = ',ScriptName,'.'
		,FunctionName,'(',ParameterString,');sys.stdout = save_out; print str(ret)'],CallArgu),
	setup_call_cleanup(
    process_create(path(python),['-c', CallArgu],[stdout(pipe(Out))]),
    read_lines(Out,OLines),
    close(Out)),
    atomic_list_concat(OLines,OLine), % To also get strings over more than one line
    name(OLine,ReturnAsChrList),
    delete(ReturnAsChrList,32,ReturnNoBlank),
    name(ReturnPre,ReturnNoBlank),
    string_list_to_list(ReturnPre,Return),
    working_directory(_,OldPath),!.

%% py_call(+PathTo:string, +ScriptName:string, +FunctionName:string, +Parameter:list, ?ReturnTyped) is semidet.
%
% Predicate to call a function of a python file. It returns the values with the right types.
%
% @param PathTo The path to the python file
% @param ScriptName The name of the python script
% @param FunctionName The function to be called
% @param Parameter The Parameter for the python function
% @param ReturnTyped The return value of the python function
%
py_call(PathTo,ScriptName,FunctionName,Parameter,ReturnTyped) :-
	py_call_base(PathTo,ScriptName,FunctionName,Parameter,Return),
	return_true_type(Return,ReturnTyped).

%% py_call(+ScriptName:string, +FunctionName:string, +Parameter:list, ?ReturnTyped) is semidet.
%
% Predicate to call a function of a python file. The file need to be placed 
% at ../scripts from this prolog source lies. It returns the values with the right types.
%
% @param ScriptName The name of the python script
% @param FunctionName The function to be called
% @param Parameter The Parameter for the python function
% @param ReturnTyped The return value of the python function
%
py_call(ScriptName,FunctionName,Parameter,ReturnTyped) :-
	python_path(Path),
	atomic_list_concat([Path,'/',ScriptName,'.py'],FullPath),
	(exists_file(FullPath) -> py_call(Path,ScriptName,FunctionName,Parameter,ReturnTyped)).

%% add_py_path(+Path) is det.
%
% Add paths to the python files
%
% @param Path Path to the python file
%
add_py_path(Path) :-
	assert(python_path(Path)).

%% remove_py_path(+Path) is det.
%
% Removes the paths to the python files
%
% @param Path Path to the python file
%
remove_py_path(Path) :-
	retract(python_path(Path)).

%%%%%%%%%%%%%%% Help Predicates %%%%%%%%%%%%%%%%%%%%%%

%% create_parameter_string(Parameter,ParameterString) is semidet.
%
create_parameter_string(Parameter,ParameterString) :-
  is_list(Parameter),
  maplist(create_parameter_string,Parameter,ParameterTrueTyped),
  atomic_list_concat(ParameterTrueTyped,',',Clist),
  atom_string(Clist,ParameterString).

create_parameter_string(Parameter,ParameterString) :-
  atom(Parameter),
  name(Parameter,ParList),
  reverse([39|ParList],Reversed),
  reverse([39|Reversed], ReversedFull),
  name(ParameterString,ReversedFull).

create_parameter_string(Parameter,_) :-
  number(Parameter).  

%% return_true_type(+Input:string, -TypedInput) is semidet.
%
% Predicate to get the input in the right type
%
% @param Input A String
% @param TypedInput The value of the string in the right type
%
return_true_type(Input, TypedInput) :-
	(is_list(Input) -> maplist(return_true_type,Input,TypedInput));
	(atom(Input)->
    (atom_number(Input,TypedInput)-> 
      true;
      TypedInput=Input);
    TypedInput=Input).	


%% string_list_to_list(Original, List) is semidet.
%
% Creates from a string in the form of a python list a list, e.g: '[1,2,[1,2]]' will bind List to ['1','2',['1','2']]
%
% @param Original A String
% @param TypedInput The value of the string in the right type
%
string_list_to_list(Original,List) :-
  is_list(Original),
  max_depth(Original,NumOfOList),
  maplist(string_list_to_list_one,Original,NewList),
  max_depth(NewList,NumOfNList),
  (NumOfOList=\=NumOfNList ->
    maplist(string_list_to_list,NewList,List);
    List = NewList).

string_list_to_list(Original,List) :-
  atom(Original),
  string_list_to_list_one(Original,NewList),
  (is_list(NewList) ->
    string_list_to_list(NewList,List);
    List=Original).
  
%% string_list_to_list(Original, List) is semidet.
%
% Help-function for string_list_to_list. It will not translate nested lists:
% e.g. '[1,2,[1,2]]' will bind List to ['1','2','[1,2]']
%
% @param Original A String
% @param TypedInput The value of the string in the right type
%
string_list_to_list_one(Original,List) :-
  name(Original,OriginalStr),
  string_list_to_list_one(OriginalStr,OriginalStr,0,[],[],List),!.

string_list_to_list_one(Original,RestStr,0,CurrString,CurrList,List) :-
  RestStr = [FirstChar|Rest],
  (FirstChar=91
    -> string_list_to_list_one(Original,Rest,1,CurrString,CurrList,List);(name(OriginalAtom,Original),List=OriginalAtom)).

string_list_to_list_one(Original,RestStr,1,CurrString,CurrList,List) :-
  RestStr = [FirstChar|Rest],
  ((FirstChar=91
    -> append(CurrString,[91],NewString),string_list_to_list_one(Original,Rest,2,NewString,CurrList,List));
    (FirstChar=93
      -> (Rest=[]->(name(CurrAtom,CurrString),append(CurrList,[CurrAtom],NewList),List=NewList);(name(OriginalAtom,Original),List=OriginalAtom)));
    (FirstChar=44 
      -> name(CurrAtom,CurrString),append(CurrList,[CurrAtom],NewList),string_list_to_list_one(Original,Rest,1,[],NewList,List));
    (append(CurrString,[FirstChar],NewString),string_list_to_list_one(Original,Rest,1,NewString,CurrList,List))
  ).

string_list_to_list_one(Original,RestStr,CountBracket,CurrString,CurrList,List) :-
  CountBracket >= 2,
  RestStr = [FirstChar|Rest],
  ((FirstChar=91
    -> append(CurrString,[91],NewString),NewCountBracket is CountBracket + 1,string_list_to_list_one(Original,Rest,NewCountBracket,NewString,CurrList,List));
  (FirstChar=93
    -> append(CurrString,[93],NewString),NewCountBracket is CountBracket - 1,string_list_to_list_one(Original,Rest,NewCountBracket,NewString,CurrList,List));
  (append(CurrString,[FirstChar],NewString),string_list_to_list_one(Original,Rest,CountBracket,NewString,CurrList,List))
  ).

%% max_depth(+List,-MaxDepth) is det
% 
% Calculates the max. depth of a nested list
%
% @param List
% @MaxDepth The maximum depth of the list
max_depth(List,MaxDepth) :-
  (is_list(List) -> 
    (maplist(max_depth,List,DepthList),
    max_list(DepthList,MDepth),
    MaxDepth is MDepth + 1)
  ;MaxDepth = 0).

max_depth([],MaxDepth) :-
  MaxDepth = 0.

%% read_lines(+Out, -Lines) is semidet.
%
% Reads lines from the stdoutput
%
% @param Input A String
% @param TypedInput The value of the string in the right type
%
read_lines(Out, Lines) :-
        read_line_to_codes(Out, Line1),
        read_lines(Line1, Out, Lines).
read_lines(end_of_file, _, []) :- !.
read_lines(Codes, Out, [Line|Lines]) :-
        atom_codes(Line, Codes),
        read_line_to_codes(Out, Line2),
        read_lines(Line2, Out, Lines).

%% remove_char(+String:string, +Char:int, -NewString) is det
%
% Simply removes a defined char from a string
%
remove_char(String, Char, NewString) :-
	name(String,CharList),
	delete(CharList,Char,CleanedCharList),
	name(NewString,CleanedCharList).