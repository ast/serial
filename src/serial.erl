-module(serial).
-include_lib("eunit/include/eunit.hrl").

-export([open/5, poll/1, close/1, read/1, write/2]).
-on_load(init/0).

init() ->
  PrivDir = code:priv_dir(?MODULE),
  NifPath = filename:absname_join(PrivDir, "serial"),
  ok = erlang:load_nif(NifPath, 0).

open(_Path, _Speed, _Parity, _Databits, _Stopbits) ->
  exit(nif_library_not_loaded).
poll(_Ref) ->
  exit(nif_library_not_loaded).
close(_REf) ->
  exit(nif_library_not_loaded).
read(_Ref) ->
  exit(nif_library_not_loaded).
write(_Ref, _Bin) ->
  exit(nif_library_not_loaded).

%% tests

loop(SerialRef) ->
  serial:poll(SerialRef),
  receive
    {select, SerialRef, undefined, ready_input} ->
      {ok, Bin} = serial:read(SerialRef),
      io:format(user, "~w~n~n", [Bin]),
      loop(SerialRef)
  end.

serial_test() ->
  {ok, SerialRef} = serial:open("/dev/cu.usbserial-A50285BI"),
  loop(SerialRef),
  ok = serial:close(SerialRef),
  io:format(user, "done..", []),
  ok.