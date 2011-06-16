Attribute VB_Name = "Module1"

Sub Legs_Click()

' si dimensionionano tutte le variabili..
Dim DestFile As String
Dim FileNum As Integer
Dim ColumnCount As Integer
Dim RowCount As Integer
Const DELIMITER As String = " "

Set MiaZona = Sheets(2).Range("A10:M150")
' si imposta la Inputbox per la richiesta del percorso completo dove salvare
'DestFile = InputBox("Inserisci il nome con cui salvare" _
'& Chr(10) & "(Completo di path):", "Esporta File di testo")
DestFile = (Sheets(1).Cells(96, 2).Text & "\" & "icub_legs_safe.ini")
' set the current up to 3Ampere
Sheets(2).Cells(5, 2) = 3000
' si ottiene il numero libero disponibile dell'handle (finestra di livello superiore della finestra di 'Excel)
FileNum = FreeFile()
' si imposta un controllo di errori
On Error Resume Next
' si predispone l'apertura del file di destinatione  come output.
Open DestFile For Output As #FileNum
' in caso di errore si avvisa con messaggio e si chiude la richiesta di apertura file (Open)
If Err <> 0 Then
MsgBox "Impossibile aprire il file " & DestFile
End
End If
' si reimposta l'errore a zero (nullo)
On Error GoTo 0
' si inizia il ciclo dalla prima all'ultima riga della selezione
For RowCount = 1 To MiaZona.Rows.Count
    ' si inizia il ciclo dalla prima all'ultima colonna della For ColumnCount = 1 To MiaZonaColumns.Count
    For ColumnCount = 1 To MiaZona.Columns.Count
    ' si scrive il testo della cella corrente nel file aperto
        If MiaZona.Cells(RowCount, ColumnCount).Text = "" Then
        Else
            Print #FileNum, MiaZona.Cells(RowCount, ColumnCount).Text, DELIMETER;
                ' si controlla se la cella è nell'ultima colonna della selezione.
        End If
        If ColumnCount = MiaZona.Columns.Count Then
              ' se è nell'ultima colonna, si lascia uno spazio vuoto
               Print #FileNum, DELIMITER
            Else  'altrimenti
                ' si scrive il separatore punto e virgola (tra doppi apici)
                Print #FileNum, "";
            End If
    ' si continua l'iterazione alla prossima colonna
Next ColumnCount
' si continua l'iterazione alla prossima riga
Next RowCount
' alla fine dei cicli si chiude il file di destinazione salvandolo
Close #FileNum

DestFile = (Sheets(1).Cells(96, 2).Text & "\" & "icub_legs.ini")
' set the current up to 3Ampere
Sheets(2).Cells(5, 2) = 7000
' si ottiene il numero libero disponibile dell'handle (finestra di livello superiore della finestra di 'Excel)
FileNum = FreeFile()
' si imposta un controllo di errori
On Error Resume Next
' si predispone l'apertura del file di destinatione  come output.
Open DestFile For Output As #FileNum
' in caso di errore si avvisa con messaggio e si chiude la richiesta di apertura file (Open)
If Err <> 0 Then
MsgBox "Impossibile aprire il file " & DestFile
End
End If
' si reimposta l'errore a zero (nullo)
On Error GoTo 0
' si inizia il ciclo dalla prima all'ultima riga della selezione
For RowCount = 1 To MiaZona.Rows.Count
    ' si inizia il ciclo dalla prima all'ultima colonna della For ColumnCount = 1 To MiaZonaColumns.Count
    For ColumnCount = 1 To MiaZona.Columns.Count
    ' si scrive il testo della cella corrente nel file aperto
        If MiaZona.Cells(RowCount, ColumnCount).Text = "" Then
        Else
            Print #FileNum, MiaZona.Cells(RowCount, ColumnCount).Text, DELIMETER;
                ' si controlla se la cella è nell'ultima colonna della selezione.
        End If
        If ColumnCount = MiaZona.Columns.Count Then
              ' se è nell'ultima colonna, si lascia uno spazio vuoto
               Print #FileNum, DELIMITER
            Else  'altrimenti
                ' si scrive il separatore punto e virgola (tra doppi apici)
                Print #FileNum, "";
            End If
    ' si continua l'iterazione alla prossima colonna
Next ColumnCount
' si continua l'iterazione alla prossima riga
Next RowCount
' alla fine dei cicli si chiude il file di destinazione salvandolo
Close #FileNum

End Sub


Sub HeadTorso_Click()

' si dimensionionano tutte le variabili..
Dim DestFile As String
Dim FileNum As Integer
Dim ColumnCount As Integer
Dim RowCount As Integer
Const DELIMITER As String = " "

Set MiaZona = Sheets(7).Range("A10:M150")
' si imposta la Inputbox per la richiesta del percorso completo dove salvare
'DestFile = InputBox("Inserisci il nome con cui salvare" _
'& Chr(10) & "(Completo di path):", "Esporta File di testo")
DestFile = (Sheets(1).Cells(96, 2).Text & "\" & "icub_head_torso_safe.ini")
' set the current up to 3Ampere
Sheet7.Cells(5, 2) = 3000
' si ottiene il numero libero disponibile dell'handle (finestra di livello superiore della finestra di 'Excel)
FileNum = FreeFile()
' si imposta un controllo di errori
On Error Resume Next
' si predispone l'apertura del file di destinatione  come output.
Open DestFile For Output As #FileNum
' in caso di errore si avvisa con messaggio e si chiude la richiesta di apertura file (Open)
If Err <> 0 Then
MsgBox "Impossibile aprire il file " & DestFile
End
End If
' si reimposta l'errore a zero (nullo)
On Error GoTo 0
' si inizia il ciclo dalla prima all'ultima riga della selezione
For RowCount = 1 To MiaZona.Rows.Count
    ' si inizia il ciclo dalla prima all'ultima colonna della For ColumnCount = 1 To MiaZonaColumns.Count
    For ColumnCount = 1 To MiaZona.Columns.Count
    ' si scrive il testo della cella corrente nel file aperto
        If MiaZona.Cells(RowCount, ColumnCount).Text = "" Then
        Else
            Print #FileNum, MiaZona.Cells(RowCount, ColumnCount).Text, DELIMETER;
                ' si controlla se la cella è nell'ultima colonna della selezione.
        End If
        If ColumnCount = MiaZona.Columns.Count Then
              ' se è nell'ultima colonna, si lascia uno spazio vuoto
               Print #FileNum, DELIMITER
            Else  'altrimenti
                ' si scrive il separatore punto e virgola (tra doppi apici)
                Print #FileNum, "";
            End If
    ' si continua l'iterazione alla prossima colonna
Next ColumnCount
' si continua l'iterazione alla prossima riga
Next RowCount

' alla fine dei cicli si chiude il file di destinazione salvandolo
Close #FileNum

DestFile = (Sheets(1).Cells(96, 2).Text & "\" & "icub_head_torso.ini")
' set the current up to 3Ampere
Sheet7.Cells(5, 2) = 7000
' si ottiene il numero libero disponibile dell'handle (finestra di livello superiore della finestra di 'Excel)
FileNum = FreeFile()
' si imposta un controllo di errori
On Error Resume Next
' si predispone l'apertura del file di destinatione  come output.
Open DestFile For Output As #FileNum
' in caso di errore si avvisa con messaggio e si chiude la richiesta di apertura file (Open)
If Err <> 0 Then
MsgBox "Impossibile aprire il file " & DestFile
End
End If
' si reimposta l'errore a zero (nullo)
On Error GoTo 0
' si inizia il ciclo dalla prima all'ultima riga della selezione
For RowCount = 1 To MiaZona.Rows.Count
    ' si inizia il ciclo dalla prima all'ultima colonna della For ColumnCount = 1 To MiaZonaColumns.Count
    For ColumnCount = 1 To MiaZona.Columns.Count
    ' si scrive il testo della cella corrente nel file aperto
        If MiaZona.Cells(RowCount, ColumnCount).Text = "" Then
        Else
            Print #FileNum, MiaZona.Cells(RowCount, ColumnCount).Text, DELIMETER;
                ' si controlla se la cella è nell'ultima colonna della selezione.
        End If
        If ColumnCount = MiaZona.Columns.Count Then
              ' se è nell'ultima colonna, si lascia uno spazio vuoto
               Print #FileNum, DELIMITER
            Else  'altrimenti
                ' si scrive il separatore punto e virgola (tra doppi apici)
                Print #FileNum, "";
            End If
    ' si continua l'iterazione alla prossima colonna
Next ColumnCount
' si continua l'iterazione alla prossima riga
Next RowCount

' alla fine dei cicli si chiude il file di destinazione salvandolo
Close #FileNum

End Sub
Sub LeftArm_Click()

' si dimensionionano tutte le variabili..
Dim DestFile As String
Dim FileNum As Integer
Dim ColumnCount As Integer
Dim RowCount As Integer
Const DELIMITER As String = " "

Set MiaZona = Sheets(3).Range("A10:M150")
' si imposta la Inputbox per la richiesta del percorso completo dove salvare
'DestFile = InputBox("Inserisci il nome con cui salvare" _
'& Chr(10) & "(Completo di path):", "Esporta File di testo")
DestFile = (Sheets(1).Cells(96, 2).Text & "\" & "icub_left_arm_safe.ini")
' set the current up to 3Ampere
Sheets(3).Cells(5, 2) = 3000
' si ottiene il numero libero disponibile dell'handle (finestra di livello superiore della finestra di 'Excel)
FileNum = FreeFile()
' si imposta un controllo di errori
On Error Resume Next
' si predispone l'apertura del file di destinatione  come output.
Open DestFile For Output As #FileNum
' in caso di errore si avvisa con messaggio e si chiude la richiesta di apertura file (Open)
If Err <> 0 Then
MsgBox "Impossibile aprire il file " & DestFile
End
End If
' si reimposta l'errore a zero (nullo)
On Error GoTo 0
' si inizia il ciclo dalla prima all'ultima riga della selezione
For RowCount = 1 To MiaZona.Rows.Count
    ' si inizia il ciclo dalla prima all'ultima colonna della For ColumnCount = 1 To MiaZonaColumns.Count
    For ColumnCount = 1 To MiaZona.Columns.Count
    ' si scrive il testo della cella corrente nel file aperto
        If MiaZona.Cells(RowCount, ColumnCount).Text = "" Then
        Else
            Print #FileNum, MiaZona.Cells(RowCount, ColumnCount).Text, DELIMETER;
                ' si controlla se la cella è nell'ultima colonna della selezione.
        End If
        If ColumnCount = MiaZona.Columns.Count Then
              ' se è nell'ultima colonna, si lascia uno spazio vuoto
               Print #FileNum, DELIMITER
            Else  'altrimenti
                ' si scrive il separatore punto e virgola (tra doppi apici)
                Print #FileNum, "";
            End If
    ' si continua l'iterazione alla prossima colonna
Next ColumnCount
' si continua l'iterazione alla prossima riga
Next RowCount
' alla fine dei cicli si chiude il file di destinazione salvandolo
Close #FileNum

DestFile = (Sheets(1).Cells(96, 2).Text & "\" & "icub_left_arm.ini")
' set the current up to 3Ampere
Sheets(3).Cells(5, 2) = 7000
' si ottiene il numero libero disponibile dell'handle (finestra di livello superiore della finestra di 'Excel)
FileNum = FreeFile()
' si imposta un controllo di errori
On Error Resume Next
' si predispone l'apertura del file di destinatione  come output.
Open DestFile For Output As #FileNum
' in caso di errore si avvisa con messaggio e si chiude la richiesta di apertura file (Open)
If Err <> 0 Then
MsgBox "Impossibile aprire il file " & DestFile
End
End If
' si reimposta l'errore a zero (nullo)
On Error GoTo 0
' si inizia il ciclo dalla prima all'ultima riga della selezione
For RowCount = 1 To MiaZona.Rows.Count
    ' si inizia il ciclo dalla prima all'ultima colonna della For ColumnCount = 1 To MiaZonaColumns.Count
    For ColumnCount = 1 To MiaZona.Columns.Count
    ' si scrive il testo della cella corrente nel file aperto
        If MiaZona.Cells(RowCount, ColumnCount).Text = "" Then
        Else
            Print #FileNum, MiaZona.Cells(RowCount, ColumnCount).Text, DELIMETER;
                ' si controlla se la cella è nell'ultima colonna della selezione.
        End If
        If ColumnCount = MiaZona.Columns.Count Then
              ' se è nell'ultima colonna, si lascia uno spazio vuoto
               Print #FileNum, DELIMITER
            Else  'altrimenti
                ' si scrive il separatore punto e virgola (tra doppi apici)
                Print #FileNum, "";
            End If
    ' si continua l'iterazione alla prossima colonna
Next ColumnCount
' si continua l'iterazione alla prossima riga
Next RowCount
' alla fine dei cicli si chiude il file di destinazione salvandolo
Close #FileNum

End Sub
Sub LeftHand_Click()

' si dimensionionano tutte le variabili..
Dim DestFile As String
Dim FileNum As Integer
Dim ColumnCount As Integer
Dim RowCount As Integer
Const DELIMITER As String = " "

Set MiaZona = Sheets(4).Range("A6:M150")
DestFile = (Sheets(1).Cells(96, 2).Text & "\" & "icub_left_hand_safe.ini")
' set the current up to 3Ampere
Sheets(4).Cells(5, 2) = 3000
' si ottiene il numero libero disponibile dell'handle (finestra di livello superiore della finestra di 'Excel)
FileNum = FreeFile()
' si imposta un controllo di errori
On Error Resume Next
' si predispone l'apertura del file di destinatione  come output.
Open DestFile For Output As #FileNum
' in caso di errore si avvisa con messaggio e si chiude la richiesta di apertura file (Open)
If Err <> 0 Then
MsgBox "Impossibile aprire il file " & DestFile
End
End If
' si reimposta l'errore a zero (nullo)
On Error GoTo 0
' si inizia il ciclo dalla prima all'ultima riga della selezione
For RowCount = 1 To MiaZona.Rows.Count
    ' si inizia il ciclo dalla prima all'ultima colonna della For ColumnCount = 1 To MiaZonaColumns.Count
    For ColumnCount = 1 To MiaZona.Columns.Count
    ' si scrive il testo della cella corrente nel file aperto
        If MiaZona.Cells(RowCount, ColumnCount).Text = "" Then
        Else
            Print #FileNum, MiaZona.Cells(RowCount, ColumnCount).Text, DELIMETER;
                ' si controlla se la cella è nell'ultima colonna della selezione.
        End If
        If ColumnCount = MiaZona.Columns.Count Then
              ' se è nell'ultima colonna, si lascia uno spazio vuoto
               Print #FileNum, DELIMITER
            Else  'altrimenti
                ' si scrive il separatore punto e virgola (tra doppi apici)
                Print #FileNum, "";
            End If
    ' si continua l'iterazione alla prossima colonna
Next ColumnCount
' si continua l'iterazione alla prossima riga
Next RowCount
' alla fine dei cicli si chiude il file di destinazione salvandolo
Close #FileNum

DestFile = (Sheets(1).Cells(96, 2).Text & "\" & "icub_left_hand.ini")
' set the current up to 3Ampere
Sheets(4).Cells(5, 2) = 7000
' si ottiene il numero libero disponibile dell'handle (finestra di livello superiore della finestra di 'Excel)
FileNum = FreeFile()
' si imposta un controllo di errori
On Error Resume Next
' si predispone l'apertura del file di destinatione  come output.
Open DestFile For Output As #FileNum
' in caso di errore si avvisa con messaggio e si chiude la richiesta di apertura file (Open)
If Err <> 0 Then
MsgBox "Impossibile aprire il file " & DestFile
End
End If
' si reimposta l'errore a zero (nullo)
On Error GoTo 0
' si inizia il ciclo dalla prima all'ultima riga della selezione
For RowCount = 1 To MiaZona.Rows.Count
    ' si inizia il ciclo dalla prima all'ultima colonna della For ColumnCount = 1 To MiaZonaColumns.Count
    For ColumnCount = 1 To MiaZona.Columns.Count
    ' si scrive il testo della cella corrente nel file aperto
        If MiaZona.Cells(RowCount, ColumnCount).Text = "" Then
        Else
            Print #FileNum, MiaZona.Cells(RowCount, ColumnCount).Text, DELIMETER;
                ' si controlla se la cella è nell'ultima colonna della selezione.
        End If
        If ColumnCount = MiaZona.Columns.Count Then
              ' se è nell'ultima colonna, si lascia uno spazio vuoto
               Print #FileNum, DELIMITER
            Else  'altrimenti
                ' si scrive il separatore punto e virgola (tra doppi apici)
                Print #FileNum, "";
            End If
    ' si continua l'iterazione alla prossima colonna
Next ColumnCount
' si continua l'iterazione alla prossima riga
Next RowCount
' alla fine dei cicli si chiude il file di destinazione salvandolo
Close #FileNum

End Sub
Sub RightArm_click()


' si dimensionionano tutte le variabili..
Dim DestFile As String
Dim FileNum As Integer
Dim ColumnCount As Integer
Dim RowCount As Integer
Const DELIMITER As String = " "

Set MiaZona = Sheets(5).Range("A10:M150")
' si imposta la Inputbox per la richiesta del percorso completo dove salvare
'DestFile = InputBox("Inserisci il nome con cui salvare" _
'& Chr(10) & "(Completo di path):", "Esporta File di testo")
DestFile = (Sheets(1).Cells(96, 2).Text & "\" & "icub_right_arm_safe.ini")
' set the current up to 3Ampere
Sheets(5).Cells(5, 2) = 3000
' si ottiene il numero libero disponibile dell'handle (finestra di livello superiore della finestra di 'Excel)
FileNum = FreeFile()
' si imposta un controllo di errori
On Error Resume Next
' si predispone l'apertura del file di destinatione  come output.
Open DestFile For Output As #FileNum
' in caso di errore si avvisa con messaggio e si chiude la richiesta di apertura file (Open)
If Err <> 0 Then
MsgBox "Impossibile aprire il file " & DestFile
End
End If
' si reimposta l'errore a zero (nullo)
On Error GoTo 0
' si inizia il ciclo dalla prima all'ultima riga della selezione
For RowCount = 1 To MiaZona.Rows.Count
    ' si inizia il ciclo dalla prima all'ultima colonna della For ColumnCount = 1 To MiaZonaColumns.Count
    For ColumnCount = 1 To MiaZona.Columns.Count
    ' si scrive il testo della cella corrente nel file aperto
        If MiaZona.Cells(RowCount, ColumnCount).Text = "" Then
        Else
            Print #FileNum, MiaZona.Cells(RowCount, ColumnCount).Text, DELIMETER;
                ' si controlla se la cella è nell'ultima colonna della selezione.
        End If
        If ColumnCount = MiaZona.Columns.Count Then
              ' se è nell'ultima colonna, si lascia uno spazio vuoto
               Print #FileNum, DELIMITER
            Else  'altrimenti
                ' si scrive il separatore punto e virgola (tra doppi apici)
                Print #FileNum, "";
            End If
    ' si continua l'iterazione alla prossima colonna
Next ColumnCount
' si continua l'iterazione alla prossima riga
Next RowCount
' alla fine dei cicli si chiude il file di destinazione salvandolo
Close #FileNum

DestFile = (Sheets(1).Cells(96, 2).Text & "\" & "icub_right_arm.ini")
' set the current up to 3Ampere
Sheets(5).Cells(5, 2) = 7000
' si ottiene il numero libero disponibile dell'handle (finestra di livello superiore della finestra di 'Excel)
FileNum = FreeFile()
' si imposta un controllo di errori
On Error Resume Next
' si predispone l'apertura del file di destinatione  come output.
Open DestFile For Output As #FileNum
' in caso di errore si avvisa con messaggio e si chiude la richiesta di apertura file (Open)
If Err <> 0 Then
MsgBox "Impossibile aprire il file " & DestFile
End
End If
' si reimposta l'errore a zero (nullo)
On Error GoTo 0
' si inizia il ciclo dalla prima all'ultima riga della selezione
For RowCount = 1 To MiaZona.Rows.Count
    ' si inizia il ciclo dalla prima all'ultima colonna della For ColumnCount = 1 To MiaZonaColumns.Count
    For ColumnCount = 1 To MiaZona.Columns.Count
    ' si scrive il testo della cella corrente nel file aperto
        If MiaZona.Cells(RowCount, ColumnCount).Text = "" Then
        Else
            Print #FileNum, MiaZona.Cells(RowCount, ColumnCount).Text, DELIMETER;
                ' si controlla se la cella è nell'ultima colonna della selezione.
        End If
        If ColumnCount = MiaZona.Columns.Count Then
              ' se è nell'ultima colonna, si lascia uno spazio vuoto
               Print #FileNum, DELIMITER
            Else  'altrimenti
                ' si scrive il separatore punto e virgola (tra doppi apici)
                Print #FileNum, "";
            End If
    ' si continua l'iterazione alla prossima colonna
Next ColumnCount
' si continua l'iterazione alla prossima riga
Next RowCount
' alla fine dei cicli si chiude il file di destinazione salvandolo
Close #FileNum

End Sub

Sub RightHand_Click()


' si dimensionionano tutte le variabili..
Dim DestFile As String
Dim FileNum As Integer
Dim ColumnCount As Integer
Dim RowCount As Integer
Const DELIMITER As String = " "

Set MiaZona = Sheets(6).Range("A6:M150")
DestFile = (Sheets(1).Cells(96, 2).Text & "\" & "icub_right_hand_safe.ini")
' si ottiene il numero libero disponibile dell'handle (finestra di livello superiore della finestra di 'Excel)
FileNum = FreeFile()
' si imposta un controllo di errori
On Error Resume Next
' si predispone l'apertura del file di destinatione  come output.
Open DestFile For Output As #FileNum
' in caso di errore si avvisa con messaggio e si chiude la richiesta di apertura file (Open)
If Err <> 0 Then
MsgBox "Impossibile aprire il file " & DestFile
End
End If
' si reimposta l'errore a zero (nullo)
On Error GoTo 0
' si inizia il ciclo dalla prima all'ultima riga della selezione
For RowCount = 1 To MiaZona.Rows.Count
    ' si inizia il ciclo dalla prima all'ultima colonna della For ColumnCount = 1 To MiaZonaColumns.Count
    For ColumnCount = 1 To MiaZona.Columns.Count
    ' si scrive il testo della cella corrente nel file aperto
        If MiaZona.Cells(RowCount, ColumnCount).Text = "" Then
        Else
            Print #FileNum, MiaZona.Cells(RowCount, ColumnCount).Text, DELIMETER;
                ' si controlla se la cella è nell'ultima colonna della selezione.
        End If
        If ColumnCount = MiaZona.Columns.Count Then
              ' se è nell'ultima colonna, si lascia uno spazio vuoto
               Print #FileNum, DELIMITER
            Else  'altrimenti
                ' si scrive il separatore punto e virgola (tra doppi apici)
                Print #FileNum, "";
            End If
    ' si continua l'iterazione alla prossima colonna
Next ColumnCount
' si continua l'iterazione alla prossima riga
Next RowCount
' alla fine dei cicli si chiude il file di destinazione salvandolo
Close #FileNum

DestFile = (Sheets(1).Cells(96, 2).Text & "\" & "icub_right_hand.ini")
' si ottiene il numero libero disponibile dell'handle (finestra di livello superiore della finestra di 'Excel)
FileNum = FreeFile()
' si imposta un controllo di errori
On Error Resume Next
' si predispone l'apertura del file di destinatione  come output.
Open DestFile For Output As #FileNum
' in caso di errore si avvisa con messaggio e si chiude la richiesta di apertura file (Open)
If Err <> 0 Then
MsgBox "Impossibile aprire il file " & DestFile
End
End If
' si reimposta l'errore a zero (nullo)
On Error GoTo 0
' si inizia il ciclo dalla prima all'ultima riga della selezione
For RowCount = 1 To MiaZona.Rows.Count
    ' si inizia il ciclo dalla prima all'ultima colonna della For ColumnCount = 1 To MiaZonaColumns.Count
    For ColumnCount = 1 To MiaZona.Columns.Count
    ' si scrive il testo della cella corrente nel file aperto
        If MiaZona.Cells(RowCount, ColumnCount).Text = "" Then
        Else
            Print #FileNum, MiaZona.Cells(RowCount, ColumnCount).Text, DELIMETER;
                ' si controlla se la cella è nell'ultima colonna della selezione.
        End If
        If ColumnCount = MiaZona.Columns.Count Then
              ' se è nell'ultima colonna, si lascia uno spazio vuoto
               Print #FileNum, DELIMITER
            Else  'altrimenti
                ' si scrive il separatore punto e virgola (tra doppi apici)
                Print #FileNum, "";
            End If
    ' si continua l'iterazione alla prossima colonna
Next ColumnCount
' si continua l'iterazione alla prossima riga
Next RowCount

' alla fine dei cicli si chiude il file di destinazione salvandolo
Close #FileNum

End Sub

Sub robotMotorGui()

' si dimensionionano tutte le variabili..
Dim DestFile As String
Dim FileNum As Integer
Dim ColumnCount As Integer
Dim RowCount As Integer
Const DELIMITER As String = " "

Set MiaZona = Sheets(8).Range("A1:R150")
DestFile = (Sheets(1).Cells(96, 2).Text & "\" & "robotMotorGui.ini")
' si ottiene il numero libero disponibile dell'handle (finestra di livello superiore della finestra di 'Excel)
FileNum = FreeFile()
' si imposta un controllo di errori
On Error Resume Next
' si predispone l'apertura del file di destinatione  come output.
Open DestFile For Output As #FileNum
' in caso di errore si avvisa con messaggio e si chiude la richiesta di apertura file (Open)
If Err <> 0 Then
MsgBox "Impossibile aprire il file " & DestFile
End
End If
' si reimposta l'errore a zero (nullo)
On Error GoTo 0
' si inizia il ciclo dalla prima all'ultima riga della selezione
For RowCount = 1 To MiaZona.Rows.Count
    ' si inizia il ciclo dalla prima all'ultima colonna della For ColumnCount = 1 To MiaZonaColumns.Count
    For ColumnCount = 1 To MiaZona.Columns.Count
    ' si scrive il testo della cella corrente nel file aperto
        If MiaZona.Cells(RowCount, ColumnCount).Text = "" Then
        Else
            Print #FileNum, MiaZona.Cells(RowCount, ColumnCount).Text, DELIMETER;
                ' si controlla se la cella è nell'ultima colonna della selezione.
        End If
        If ColumnCount = MiaZona.Columns.Count Then
              ' se è nell'ultima colonna, si lascia uno spazio vuoto
               Print #FileNum, DELIMITER
            Else  'altrimenti
                ' si scrive il separatore punto e virgola (tra doppi apici)
                Print #FileNum, "";
            End If
    ' si continua l'iterazione alla prossima colonna
Next ColumnCount
' si continua l'iterazione alla prossima riga
Next RowCount
' alla fine dei cicli si chiude il file di destinazione salvandolo
Close #FileNum

End Sub

Sub All()
'
' All Macro
'
Legs_Click
LeftArm_Click
LeftHand_Click
RightArm_click
RightHand_Click
HeadTorso_Click
robotMotorGui

End Sub







