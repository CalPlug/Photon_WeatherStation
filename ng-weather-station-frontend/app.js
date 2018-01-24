var express = require('express');
const path  = require('path');
var app     = express();
require('dotenv').config();

// Print all exceptions
process.on('uncaughtException', function (err) {
    console.log("Error: " + err);
});

var api = require('./api');

app.use('/api', api);

app.use(express.static(__dirname));

app.get('/*', function (req, res) {
    res.sendFile(path.join(__dirname,'index.html'));
});

//Start Server
var server = app.listen(process.env.PORT || 8080, function () {
    var host = server.address().address
    var port = server.address().port
  
    console.log(`Application listening... http://${host}:${port}`)
});

module.exports = app;

// var express = require('express');
// var path = require('path');
// var app = express();
// app.use(express.static(__dirname));

// app.get('/*', function (req, res) {
// res.sendFile(path.join(__dirname,'index.html'));
// });
// // catch 404 and forward to error handler
// app.use(function(req, res, next) {
// var err = new Error('Not Found');
// err.status = 404;
// next(err);
// });
// if (app.get('env') === 'development')
// {
// app.listen(3000, function () {
// console.log('Example app listening on port 3000!');
// });
// }
// else{
// app.listen(8080, function () {
// console.log('Example app listening on port 8080!');
// });
// }
// module.exports = app;