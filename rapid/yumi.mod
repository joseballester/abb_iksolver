MODULE IKSolver
  ! Configuration
  VAR string ipRobot := "10.0.2.15";
  VAR num portRobot := 5000;

  ! Frames
  PERS tooldata currentTool;
  PERS wobjdata currentWobj;

  ! Communication variables (TCP)
  VAR socketdev serverSocket;
  VAR socketdev clientSocket;

  ! Message parsing
  VAR string msg;
  VAR num params{10};
  VAR num nParams;
  VAR num ok;

  ! IK variables
  VAR robtarget cartesianPose;
  VAR robtarget currentPose;
  VAR jointtarget ikResult := [[0,0,0,0,0,0],[0,9E9,9E9,9E9,9E9,9E9]];

  PROC connectServer()
    VAR string clientIP;
    SocketCreate serverSocket;
    SocketBind serverSocket, ipRobot, portRobot;
    SocketListen serverSocket;
    WHILE SocketGetStatus(clientSocket) <> SOCKET_CONNECTED DO
      SocketAccept serverSocket, clientSocket \ClientAddress:=clientIP \Time:=WAIT_MAX;
      IF SocketGetStatus(clientSocket) <> SOCKET_CONNECTED THEN
        TPWrite "Problem serving incoming connection. Try again.";
      ENDIF
      WaitTime 0.5;
    ENDWHILE
    TPWrite "Connected to IP "+clientIP;
  ENDPROC

  PROC parseMsg()
    VAR bool auxOk;
    VAR num ind := 1;
    VAR num newInd;
    VAR num length;
    VAR num indParam := 1;
    VAR string subString;
    VAR bool end := FALSE;

    length := StrMatch(msg,1,"#");
    IF length > StrLen(msg) THEN
      ! Corrupt message
      nParams := -1;
    ELSE
      ! Load all parameters in params array
      WHILE end = FALSE DO
        newInd := StrMatch(msg, ind, " ")+1;
        IF newInd > length THEN
          end := TRUE;
        ELSE
          subString := StrPart(msg, ind, newInd-ind-1);
          auxOk := StrToVal(subString, params{indParam});
          indParam := indParam+1;
          ind := newInd;
        ENDIF
      ENDWHILE
      nParams := indParam-1;
    ENDIF
  ENDPROC

  PROC main()
    ! Start connection
    connectServer;

    ! Infinite loop
    WHILE TRUE DO
      SocketReceive clientSocket \Str:=msg; ! Receive message
      parseMsg; ! Parse message and get params

      ! 7 parameters expected (position + quaternion)
      IF nParams <> 7 THEN
        ok := 2;
      ELSEIF Abs(1.0-Sqrt(params{4}*params{4}+params{5}*params{5}+params{6}*params{6}+params{7}*params{7})) > 0.001 THEN
        ok := 0;
      ELSE
        ! Current pose to get current external axis used for IK
        currentPose := CRobT(\Tool:=currentTool \WObj:=currentWobj);

        ! Prepare target pose, including external axis angle
        cartesianPose := [[params{1},params{2},params{3}],
            NOrient([params{4},params{5},params{6},params{7}]),
            [0,0,0,0],
            [currentPose.extax.eax_a,9E9,9E9,9E9,9E9,9E9]];

        ! Compute IK result
        ok := 1;
        ikResult := CalcJointT(cartesianPose,currentTool,\WObj:=currentWobj);
      ENDIF

      ! Prepare response to client and send message
      msg := NumToStr(ok,0)+" ";
      msg := msg+NumToStr(ikResult.robax.rax_1,2)+" ";
      msg := msg+NumToStr(ikResult.robax.rax_2,2)+" ";
      msg := msg+NumToStr(ikResult.robax.rax_3,2)+" ";
      msg := msg+NumToStr(ikResult.robax.rax_4,2)+" ";
      msg := msg+NumToStr(ikResult.robax.rax_5,2)+" ";
      msg := msg+NumToStr(ikResult.robax.rax_6,2)+" ";
      msg := msg+NumToStr(ikResult.extax.eax_a,2)+ByteToStr(10\Char);
      SocketSend clientSocket \Str:=msg;
    ENDWHILE
  ERROR (LONG_JMP_ALL_ERR)
    IF ERRNO = ERR_SOCK_CLOSED THEN
      TPWrite "Socket closed by client. Reconnecting.";
      SocketClose clientSocket;
      SocketClose serverSocket;
      connectServer;
      RETRY;
    ELSE
      ! If any other error happens, just send it to client
      ok := ERRNO;
      TRYNEXT;
    ENDIF
  ENDPROC

ENDMODULE
