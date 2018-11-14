/* The command dispatcher code: */

Cmd = FDispatcherDelegate::CreateRaw(this, &FCameraCommandHandler::UecvrosFull);
Help = "Run the full UECVROS routine [X, Y, Z, p, y, r, tol, X_old, Y_old, Z_old] Return npy image unless collided";
CommandDispatcher->BindCommand("vget /uecvros/full [float] [float] [float] [float] [float] [float] [float] [float] [float] [float]", Cmd, Help);


/* The function body code: */

FExecStatus FCameraCommandHandler::UecvrosFull(const TArray<FString>& Args)
{
	if (Args.Num() == 10) // [X, Y, Z, p, y, r, collisionTolerance, X_prev, Y_prev, Z_prev]
	{
		float X = FCString::Atof(*Args[0]), Y = FCString::Atof(*Args[1]), Z = FCString::Atof(*Args[2]);
		FVector LocNew = FVector(X, Y, Z);
		
		float p = FCString::Atof(*Args[3]), y = FCString::Atof(*Args[4]), r = FCString::Atof(*Args[5]);
		FRotator RotNew = FRotator(p, y, r);
		
		float tol = FCString::Atof(*Args[6]);
		
		float Xo = FCString::Atof(*Args[7]), Yo = FCString::Atof(*Args[8]), Zo = FCString::Atof(*Args[9]);
		
		// Check for collision
		bool collided = false;
		bool sweep = tol > 0;	// Is collision enabled?
		if (sweep)
		{
			APawn* Pawn = FUE4CVServer::Get().GetPawn();
			FVector LocReal = Pawn->GetActorLocation();
			float dist = pow(pow(Xo-LocReal.X, 2.0) + pow(Yo-LocReal.Y, 2.0) + pow(Zo-LocReal.Z, 2.0), 0.5);
			collided = dist >= tol;
		}
				
		TArray<uint8> Data;
		if (!collided)
		{
			// produce images and stack binary data
			TArray<FString> CamIdArgs;
			CamIdArgs.Init(TEXT("0"), 1);		
			Data += this->GetNpyBinaryUint8Data(CamIdArgs, TEXT("lit"), 4);
			Data += this->GetNpyBinaryFloat16Data(CamIdArgs, TEXT("depth"), 1);
		}
		
		// move the actor to the new location (allowing collision if enabled)
		Pawn->SetActorLocation(LocNew, sweep, NULL, ETeleportType::TeleportPhysics);
		
		// Set new rotation
		AController* Controller = Pawn->GetController();
		Controller->ClientSetRotation(RotNew);

		// Return results
		if (collided)
		{
			return FExecStatus::OK(FString::Printf(TEXT("Collision detected at X_req=(%.3f, %.3f, %.3f), X_is=(%.3f, %.3f, %.3f), dist=%.3f, tol=%.3f"), Xo, Yo, Zo, LocReal.X, LocReal.Y, LocReal.Z, dist, tol));		// returns OK to get the message string only}
		}
		else 
		{
			return FExecStatus::Binary(Data);
		}
	}
	return FExecStatus::InvalidArgument;
}
