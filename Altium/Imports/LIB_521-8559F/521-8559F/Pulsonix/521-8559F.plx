PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//13125497/1599314/2.50/4/4/LED

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "c115_h91"
		(holeDiam 0.91)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 1.150) (shapeHeight 1.150))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 1.150) (shapeHeight 1.150))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "5218559F" (originalName "5218559F")
		(multiLayer
			(pad (padNum 1) (padStyleRef c115_h91) (pt 0.000, 0.000) (rotation 90))
			(pad (padNum 2) (padStyleRef c115_h91) (pt 0.000, -1.270) (rotation 90))
			(pad (padNum 3) (padStyleRef c115_h91) (pt 0.000, -2.540) (rotation 90))
			(pad (padNum 4) (padStyleRef c115_h91) (pt 0.000, -3.810) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0.000, -1.550) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -3.54 2.345) (pt 3.54 2.345) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 3.54 2.345) (pt 3.54 -5.445) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 3.54 -5.445) (pt -3.54 -5.445) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -3.54 -5.445) (pt -3.54 2.345) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.54 -1.905) (pt -2.54 -1.905) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(arc (pt 0, -1.905) (radius 2.54) (startAngle 180) (sweepAngle 180.0) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 2.54 -1.905) (pt 2.54 -1.905) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(arc (pt 0, -1.905) (radius 2.54) (startAngle .0) (sweepAngle 180.0) (width 0.025))
		)
		(layerContents (layerNumRef 18)
			(line (pt 0 1.295) (pt 0 1.295) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(arc (pt 0.05, 1.295) (radius 0.05) (startAngle 180.0) (sweepAngle 180.0) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt 0.1 1.295) (pt 0.1 1.295) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(arc (pt 0.05, 1.295) (radius 0.05) (startAngle .0) (sweepAngle 180.0) (width 0.1))
		)
	)
	(symbolDef "521-8559F" (originalName "521-8559F")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 3) (pt 0 mils -200 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -225 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 4) (pt 0 mils -300 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -325 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(line (pt 200 mils 100 mils) (pt 900 mils 100 mils) (width 6 mils))
		(line (pt 900 mils 100 mils) (pt 900 mils -400 mils) (width 6 mils))
		(line (pt 900 mils -400 mils) (pt 200 mils -400 mils) (width 6 mils))
		(line (pt 200 mils -400 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 950 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 950 mils 200 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "521-8559F" (originalName "521-8559F") (compHeader (numPins 4) (numParts 1) (refDesPrefix LED)
		)
		(compPin "1" (pinName "RED K") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "2" (pinName "COMMON A") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "3" (pinName "BLUE K") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "4" (pinName "GREEN K") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Unknown))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "521-8559F"))
		(attachedPattern (patternNum 1) (patternName "5218559F")
			(numPads 4)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "4")
			)
		)
		(attr "Mouser Part Number" "645-5218559F")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/Dialight/521-8559F?qs=vmHwEFxEFR%2FyCJVY9sdZTA%3D%3D")
		(attr "Manufacturer_Name" "Dialight")
		(attr "Manufacturer_Part_Number" "521-8559F")
		(attr "Description" "Standard LEDs - Through Hole 5mm RGB WC LED w4 leads")
		(attr "<Hyperlink>" "")
		(attr "<Component Height>" "8.6")
		(attr "<STEP Filename>" "521-8559F.stp")
		(attr "<STEP Offsets>" "X=0;Y=-1.88;Z=0")
		(attr "<STEP Rotation>" "X=90;Y=0;Z=0")
	)

)
