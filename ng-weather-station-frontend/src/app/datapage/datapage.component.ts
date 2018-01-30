import { Component, OnInit, AfterViewInit } from '@angular/core';
import { HttpClient, HttpHeaders } from '@angular/common/http';
import { Http, Response, Headers, RequestOptions } from '@angular/http';
import { Observable } from 'rxjs/Observable';
import {NgxChartsModule} from '@swimlane/ngx-charts';
import { Category } from '../category';

@Component({
    selector: 'app-datapage',
    templateUrl: './datapage.component.html',
    styleUrls: ['./datapage.component.css']
})
export class DatapageComponent implements OnInit {

    private apiUrl = 'api/';

    public view: any[] = [700, 400];

    categories:Category[] = [];
    cateogriesObj= {};
    public categoryData = {};
    dataready:boolean = false;

    constructor( private http: HttpClient ) { 
    }

    ngOnInit() {
        this.getCategories(data => this.populateCategories(data));
        
    }

    ngAfterViewInit() {
        console.log(this.categoryData);
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
                    } else {
                        this.categoryData[this.categories[i].name] = [{
                            series: [],
                            name:this.categories[i].children[j]
                        }];
                    }
                }
            }
        }
        this.dataready = true;
    }

    private populateSingle(ref, data, categoryName){
        const lenData = data.length;
        var arrData:Array<any> = [];
        for (var j:number = 0; j<data.length; ++j){
            arrData.push({
                name: new Date(data[j].utcTime).toLocaleString(),
                value: data[j][categoryName]
            })
        }

        ref.categoryData[categoryName] = [{
            series: arrData,
            name:categoryName
        }];

        
    }

    private populateMultiple(ref, data, categoryName, childName){
        const lenData = data.length;
        var arrData:Array<any> = [];
        for (var j:number = 0; j<data.length; ++j){
            arrData.push({
                name: new Date(data[j].utcTime).toLocaleString(),
                value: data[j][categoryName+"_"+childName]
            })
        }

        if (!ref.categoryData.hasOwnProperty(categoryName)){
            ref.categoryData[categoryName] = [];
        }
            
        ref.categoryData[categoryName].push({
            series: arrData,
            name: childName
        });
        

        ref.categoryData[categoryName] = [...ref.categoryData[categoryName]];
    }

    dataSelect(event, mainCat){
        // event = category or subcategory

        //Only worry about subcategories
        var index = this.cateogriesObj[mainCat].children.indexOf(event);
        if (index>-1 ){
            var lenChild = this.cateogriesObj[mainCat].children.length;
            console.log(lenChild);
            this.categoryData[mainCat] = [];
            for(var j:number = 0; j<lenChild; ++j){
                if (j == index){
                    this.getCategoryData(mainCat + '_' + event, data => this.populateMultiple(this, data, mainCat,event ));
                } else {
                    this.categoryData[mainCat].push( {
                        series: [],
                        name:this.cateogriesObj[mainCat].children[j]
                    } );
                    this.categoryData[mainCat] = [...this.categoryData[mainCat]];
                }
            }
            console.log(this.categoryData[mainCat]);
        }
    }

    checkDefined(category){
        return typeof(this.categoryData[category]) !== 'undefined';
    }

}
