var express = require('express');
var app = express();
var engine  = require('ejs-mate');
require('dotenv').config();

app.engine('ejs', engine);

app.set('views',__dirname + '/views');
app.set('view engine', 'ejs'); //Use EJS template engine

// Print all exceptions
process.on('uncaughtException', function (err) {
    console.log("Error: " + err);
});

//Static Public Files
app.use('/public', express.static('public'));

app.get('/', function(req, res){
    res.render('pages/testpage', {});
});


//Start Server
var server = app.listen(process.env.PORT || 8080, function () {
    var host = server.address().address
    var port = server.address().port
  
    console.log(`Application listening... http://${host}:${port}`)
});