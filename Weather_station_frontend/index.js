var express = require('express');
var app     = express();
var engine  = require('ejs-mate');
require('dotenv').config();

// Print all exceptions
process.on('uncaughtException', function (err) {
    console.log("Error: " + err);
});

var api = require('./api');

app.engine('ejs', engine);

app.set('views',__dirname + '/views');
app.set('view engine', 'ejs'); //Use EJS template engine



//Static Public Files
app.use('/public', express.static('public'));

app.use('/api', api);

app.get('/', function(req, res){
    res.render('pages/testpage', {
        'title':'Home'
    });
});

app.get('/about', function(req, res){
    res.render('pages/stdpage', {
        'title':'About',
        'content':"This is a frontend interface for viewing data recorded by the weather station."
    })
});




//Start Server
var server = app.listen(process.env.PORT || 8080, function () {
    var host = server.address().address
    var port = server.address().port
  
    console.log(`Application listening... http://${host}:${port}`)
});