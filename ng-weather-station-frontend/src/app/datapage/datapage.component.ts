import { DataService } from './../data.service';
import { Category } from './../category';
import { NewdtpComponent } from './../newdtp/newdtp.component';
import { Component, OnInit, AfterViewInit, Input, HostListener} from '@angular/core';
import { HttpClient, HttpHeaders } from '@angular/common/http';
import { Http, Response, Headers, RequestOptions } from '@angular/http';
import { Observable } from 'rxjs/Observable';


@Component({
    selector: 'app-datapage',
    templateUrl: './datapage.component.html',
    styleUrls: ['./datapage.component.css']
})
export class DatapageComponent implements OnInit {

    message_test = "hello world";

    private apiUrl = 'api/';

    categories:Category[] = [];
    public cateogriesObj= {};
    dataready:boolean = false;

    public dyOptions = {width: 'auto', animatedZooms: true, pointSize: 4, labels:["Time", '']}
    public dyCategoryData = {};

    public mostRecent = {};

    public Cname:any = "";
    public WindRelative:string = "WindDirection";

    
    //data service
    message:string;



    GetCatename(name){
        this.Cname = name;
    }

    newMessage(){
        this.data.changeMessage(this.Cname);
    }

    constructor( private http: HttpClient, private data:DataService) { 
    }

    ngOnInit() {
        this.getCategories(data => this.populateCategories(data));
        this.data.currentMessage.subscribe(message => this.message = message);
    }

    ngAfterViewInit() {
        // console.log(this.categoryData);
    }

    getCategories(onSuccess?) {
        this.http.get(this.apiUrl+'categories').subscribe(data => {
            var catData:string[] = data['categories'];
            var lenCats:number = catData.length;
            for (var i:number = 0; i<lenCats; ++i){
                var cat:string = catData[i];

                var parent:string = cat.split('_',1)[0];

                if (parent) {
                    
                    if (!(parent in this.cateogriesObj))
                        this.cateogriesObj[parent] = new Category(parent);

                    if (cat != parent){
                        var index = cat.indexOf('_');
                        var child = cat.substring(index+1);
                        this.cateogriesObj[parent].children.push(child);
                    }
                }
            }
            this.categories = Object.values(this.cateogriesObj);
            if (typeof(onSuccess) !== 'undefined') onSuccess(this.categories);
        });
    }

    getCategoryData(category, callback){
        this.http.get(this.apiUrl+'data/'+ encodeURIComponent(category)).subscribe((data) => callback(data['data']));
    }

    populateCategories(categories){
        const lenCats:number = this.categories.length;
        for (var i:number = 0; i<lenCats; ++i){
            const lenChildren = this.categories[i].children.length;
            const catName = this.categories[i].name;
            console.log('derp');
            if (!lenChildren){
                this.getCategoryData(this.categories[i].name, data => this.populateSingle(this, data, catName));
            } else {
                for(var j:number = 0; j<lenChildren; ++j){
                    if (!j){
                        const childName = this.categories[i].children[j];
                        this.getCategoryData(this.categories[i].name + '_' + this.categories[i].children[j], data => this.populateMultiple(this, data, catName,childName ));
                        break;
                    }
                }
            }
        }
        this.dataready = true;
    }

    private populateSingle(ref, data, categoryName){
        const lenData = data.length;
        var dyArrData:Array<any> = [];
        for (var j:number = 0; j<data.length; ++j){

            dyArrData.push(
                [
                    new Date(data[j].utcTime),
                    parseFloat(data[j][categoryName]) || 0
                ]
            )
            ref.mostRecent[categoryName] = parseFloat(data[j][categoryName]) || 0;
        }

        ref.dyCategoryData[categoryName] = dyArrData;

    }

    private populateMultiple(ref, data, categoryName, childName){
        const lenData = data.length;
        var dyArrData:Array<any> = [];
        for (var j:number = 0; j<data.length; ++j){
            dyArrData.push(
                [
                    new Date(data[j].utcTime),
                    parseFloat( data[j][categoryName+"_"+childName])
                ]
            )
            ref.mostRecent[categoryName] = parseFloat(data[j][categoryName+"_"+childName]) || 0;
        }
            
        ref.dyCategoryData[categoryName] = dyArrData;

        // console.log(ref.dyCategoryData);
    }

    chartSelect(mainCat, subCat){
        this.getCategoryData(mainCat+ '_' + subCat, data => this.populateMultiple(this, data, mainCat,subCat )); 
    }

    checkDefined(category){
        return typeof(this.dyCategoryData[category]) !== 'undefined';
    }

}

